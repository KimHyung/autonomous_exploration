//ros
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetPlan.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

//fontier
#include "iostream"
#include "queue"
#include "stdio.h"
#include "math.h"
#include "time.h"
//opencv
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

int dx[] = {-1,0,1,0};
int dy[] = {0,1,0,-1};
int WAYS[16][2] = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 }, { -1, 1 }, { -1, -1 }, { 1, -1 }, { 1, 1 },{ -2, 0 }, { 2, 0 }, { 0, -2}, { 0, 2 }, { -2, 2 }, { -2, -2}, { 2, -2 }, { 2, 2 }  };

struct frontier_group{
    int id;
    std::vector<int> frontier_index;
    double num_factor;
    int dist_factor;
    double region_factor;
    double info_factor;
};

class Frontiers
{
    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    image_transport::Subscriber region_sub_;

    //subscriber handler
    ros::Subscriber map_sub_;
    ros::Subscriber robot_sub_;

    //pusblisher handler
    ros::Publisher frontier_pub_;
    ros::Publisher cluster_pub_;
    ros::Publisher center_frontier_pub_;
    
    //resources-frontiers
    nav_msgs::OccupancyGridConstPtr map;
    std::vector<int> frontier;
    std::vector<int> visited;
    std::vector<int> frontier_map;
    int total_frontier_num;

    //resources-clustering
    frontier_group *clustered = new frontier_group[1000000];
    std::vector<int> valid_cluster;
    int group_id;

    //resources-region
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat segment_img;
    std::vector<int> regions;
    geometry_msgs::Point robot_pose;

    //resources-path planner
    ros::ServiceClient make_plan_;
    nav_msgs::GetPlan srv_plan;
    ros::Publisher goal_pub_;

    //check computation times
    double detect_time=0;
    double cluster_time=0;
    double cal_size_time=0;
    double cal_dist_time=0;
    double cal_region_time=0;
    double cal_region_time_=0;
    double total=0;
    int call_cn=0;

    public:
        Frontiers(const std::string& mapname): it_(n)
        {
            //subscribe
            robot_sub_ = n.subscribe("odom", 1, &Frontiers::odomCallback, this);
            region_sub_ = it_.subscribe("tagged_image", 1, &Frontiers::segmentCallback, this);
            map_sub_ = n.subscribe("map", 1, &Frontiers::mapCallback, this);
            
            //publisher
            frontier_pub_ =  n.advertise<visualization_msgs::Marker>("/frontiers", 1);
            center_frontier_pub_ =  n.advertise<visualization_msgs::Marker>("/center_frontiers", 1);
            cluster_pub_ =  n.advertise<visualization_msgs::Marker>("/region_test", 1);
            goal_pub_ = n.advertise<visualization_msgs::Marker>("/nbv_point", 1);
            
            //service
            make_plan_ = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");

        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr& robot_)
        {
            robot_pose.x = robot_->pose.pose.position.x;
            robot_pose.y = robot_->pose.pose.position.y;

        }

        void segmentCallback(const sensor_msgs::ImageConstPtr& sub_image){
            clock_t start, end;
            start = clock();
            try{
                cv_ptr = cv_bridge::toCvCopy(sub_image);
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("no image(%s)", e.what());
                return;
            }
            cv::Mat convert_img = cv_ptr->image;
            depthToCV8UC1(convert_img, segment_img);
            cv::flip(segment_img, segment_img, 0);
            int cnt=0;
            for(int i=0; i<segment_img.rows; i++){
                for(int j=0; j<segment_img.cols;j++){
                    if(segment_img.at<uchar>(j,i) != 0){
                        if(regions.empty()){
                            regions.push_back(segment_img.at<uchar>(j,i));
                        }
                        else{
                            bool check = true;
                            for(int k=0; k<regions.size(); k++){
                                if(segment_img.at<uchar>(j,i) == regions[k]){
                                    cnt++;
                                    check = false;
                                }
                            }
                            if(check){
                                regions.push_back(segment_img.at<uchar>(j,i));
                            }
                        }
                    }
                }
            }
            end = clock();
            cal_region_time = (double)(end - start)/CLOCKS_PER_SEC * 1000;
        }

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_)
        {
            map = map_;
            call_cn++;
            detect_frontier();
            cluster_frontier();
            publish_frontier();
            cal_region();
            cal_dist();
            nbv_exploration();
            print_status();
            clear_vectors();
        }

        void print_status(){
            total = detect_time+cluster_time+cal_size_time+cal_dist_time+cal_region_time+cal_region_time_;
            printf("STATUS------------------------------------\n");
            printf("[#%d]%d x %d map @ %.3f m/pix\n",call_cn, map->info.width, map->info.height, map->info.resolution);
            printf("detect %.1f ms\tcluster %.1f ms\tsize %.1f ms\n",detect_time,cluster_time,cal_size_time);
            printf("dist %.1f ms\tregion %.1f ms\t\e[1mtotal %.1f ms\e[0m\n",cal_dist_time,cal_region_time+cal_region_time_,total);
            printf("FACTORS------------------------------------\n");

            for(int i=0; i<valid_cluster.size();i++){
                printf("NO %d : [%d], dist[%d], region[%.1f], size[%.1f]\n",i,clustered[valid_cluster[i]].id, clustered[valid_cluster[i]].dist_factor, clustered[valid_cluster[i]].region_factor, clustered[valid_cluster[i]].num_factor);
            }
            printf("----------------------------------------------\n");
        }

        void detect_frontier(){
            clock_t start, end;
            start = clock();
            for(int i=0; i<map->info.width * map->info.height; i++){
                int t_x = i % map->info.width;
                int t_y = i / map->info.width;
                if(map->data[i]!=-1)
                {
                    if(t_x != map->info.width-1 && t_x != 0 && t_y != map->info.height-1 && t_y != 0 ){
                        int tmp_r = map->data[gridTomap(t_x+1,t_y,map->info.width)];
                        int tmp_l = map->data[gridTomap(t_x-1,t_y,map->info.width)];
                        int tmp_d = map->data[gridTomap(t_x,t_y-1,map->info.width)];
                        int tmp_u = map->data[gridTomap(t_x,t_y+1,map->info.width)];
                        if(tmp_r == 100 || tmp_l == 100 || tmp_d == 100 || tmp_u == 100 ){
                            frontier_map.push_back(0);
                        }
                        else{
                            if(tmp_r == -1 ||tmp_l == -1 ||tmp_d == -1 ||tmp_u == -1 ){
                                frontier_map.push_back(-1);
                                frontier.push_back(i);
                            }
                            else{
                                frontier_map.push_back(0);
                            }
                        }
                    }
                    else{
                        frontier_map.push_back(0);
                    }   
                }
                else{
                    frontier_map.push_back(0);
                }
                visited.push_back(0);
            }
            end = clock();
            detect_time = (double)(end - start)/CLOCKS_PER_SEC * 1000;
        }

        void cluster_frontier(){
            group_id = 0;
            clock_t start, end;
            start = clock();
            for(int i=0; i<frontier.size(); i++){
                bfs_search(frontier[i]);
                group_id++;
            }
            end = clock();
            cluster_time = (double)(end - start)/CLOCKS_PER_SEC*1000;
        }

        void bfs_search(int index){
            queue<int> q;
            int next,x,y;
            q.push(index);
            visited[index] = 1;
            clustered[group_id].id = group_id;
            while(!q.empty()){
                clustered[group_id].frontier_index.push_back(index);
                x = q.front() % map->info.width;
                y = q.front() / map->info.width;
                q.pop();
                //8 direction-check
                for(int i=0; i<8; i++){
                    int nx = x+WAYS[i][0];
                    int ny = y+WAYS[i][1];
                    //map size check
                    if(nx <= map->info.width-1 && nx >= 0 && ny <= map->info.height-1 && ny >= 0){
                        next = gridTomap(nx, ny, map->info.width);
                        if(frontier_map[next] == -1 && visited[next]==0){
                            visited[next]=1;
                            index = next;
                            clustered[group_id].frontier_index.push_back(index);
                            q.push(next);
                        }
                    }
                }
            }
        }

        void cal_region(){
            clock_t start, end;
            start = clock();
            if(!segment_img.empty() && regions.size() != 0){
                int r_x = (robot_pose.x - map->info.origin.position.x - map->info.resolution/2)/map->info.resolution;
                int r_y = (robot_pose.y - map->info.origin.position.y - map->info.resolution/2)/map->info.resolution;
                double robot_region =regions[segment_img.at<uchar>(r_y,r_x)];
                
                for(int i=0; i<valid_cluster.size();i++){
                    int t_x = clustered[valid_cluster[i]].frontier_index[clustered[valid_cluster[i]].frontier_index.size()/2] % map->info.width;
                    int t_y = clustered[valid_cluster[i]].frontier_index[clustered[valid_cluster[i]].frontier_index.size()/2] / map->info.width;
                    double include_region = regions[segment_img.at<uchar>(t_y,t_x)];
                    for(int j=0;j<4;j++){
                        int nx = t_x + dx[j];
                        int ny = t_y + dx[j];
                        if(map->data[gridTomap(nx,ny,map->info.width)]==0){
                            include_region = regions[segment_img.at<uchar>(ny,nx)];
                            break;
                        }
                    }
                    robot_region =regions[segment_img.at<uchar>(r_y,r_x)];
                    if(robot_region == include_region || include_region == regions[segment_img.at<uchar>(0,0)]){
                        clustered[valid_cluster[i]].region_factor = 0.5;
                    }
                }
            }
            end = clock();
            cal_region_time_ = (double)(end-start)/CLOCKS_PER_SEC * 1000;
        }

        void cal_dist(){
            clock_t start, end;
            start = clock();
            int total_waypoints=1;
            if(make_plan_){
                geometry_msgs::Point g;
                g.z=0;
                for(int i=0; i<valid_cluster.size();i++){
                    int num_frontier = clustered[valid_cluster[i]].frontier_index.size();
                    int t_x = clustered[valid_cluster[i]].frontier_index[num_frontier/2] % map->info.width;
                    int t_y = clustered[valid_cluster[i]].frontier_index[num_frontier/2] / map->info.width;
                    g.x = (t_x*map->info.resolution) + map->info.origin.position.x + map->info.resolution /2;
                    g.y = (t_y*map->info.resolution) + map->info.origin.position.y + map->info.resolution /2;
    

                    srv_plan.request.start.pose.position.x = robot_pose.x;
                    srv_plan.request.start.pose.position.y = robot_pose.y;
                    srv_plan.request.start.pose.position.z = 0;
                    srv_plan.request.goal.pose.position.x = g.x;
                    srv_plan.request.goal.pose.position.y = g.y;
                    srv_plan.request.goal.pose.position.z = g.z;
                    srv_plan.request.goal.header.frame_id = "map";
                    srv_plan.request.start.header.frame_id = "map";
                    srv_plan.request.tolerance = 1.5;

                    make_plan_.call(srv_plan);

                    int path_waypoints = srv_plan.response.plan.poses.size();

                    if(path_waypoints!=0){
                        // if(max_waypoints<path_waypoints){
                        //     max_waypoints = path_waypoints;
                        // }
                        total_waypoints = total_waypoints + path_waypoints;
                        clustered[valid_cluster[i]].dist_factor = path_waypoints;
                    }
                    else{
                        clustered[valid_cluster[i]].dist_factor = std::numeric_limits<double>::infinity();
                    }
                }
            }
            else{
                ROS_ERROR("Fail to call service to path planner");
            }
            end = clock();
            cal_dist_time = (double)(end-start)/CLOCKS_PER_SEC *1000;
        }
        
        void nbv_exploration(){
            // clock_t start, end;
            // double result;
            // start = clock();
            visualization_msgs::Marker goal_point;
            goal_point.header.frame_id = "map";
            goal_point.header.stamp = ros::Time::now();
            goal_point.ns = "goal_point";
            goal_point.action = visualization_msgs::Marker::ADD;
            goal_point.pose.orientation.w  =1.0;
            goal_point.id = 0;
            goal_point.type = visualization_msgs::Marker::POINTS;
            goal_point.scale.x = 0.3;
            goal_point.scale.y = 0.3;
            goal_point.color.r = 1.0;
            goal_point.color.g = 0.0;
            goal_point.color.b = 1.0;
            goal_point.color.a = 1.0;

            if(valid_cluster.size()!=0){
                int answer= valid_cluster[0];
                double max = clustered[valid_cluster[0]].num_factor*0.5-clustered[valid_cluster[0]].dist_factor*1e-3+clustered[valid_cluster[0]].region_factor;
                for(int i=1; i<valid_cluster.size();i++){
                    double obj =  clustered[valid_cluster[i]].num_factor*0.5-clustered[valid_cluster[i]].dist_factor*1e-3+clustered[valid_cluster[i]].region_factor;
                    if(max<obj){
                        max = obj;
                        answer = valid_cluster[i];
                    }
                }
                int t_x = clustered[answer].frontier_index[clustered[answer].frontier_index.size()/2] % map->info.width;
                int t_y = clustered[answer].frontier_index[clustered[answer].frontier_index.size()/2] / map->info.width;
                    
                geometry_msgs::Point p;
                p.x = (t_x *map->info.resolution) + map->info.origin.position.x + map->info.resolution /2;
                p.y = (t_y *map->info.resolution) + map->info.origin.position.y + map->info.resolution /2;
                p.z = 1.0;
                goal_point.points.push_back(p);
                goal_pub_.publish(goal_point);
                //send goal msg
                MoveBaseClient mc("move_base");
                while(!mc.waitForServer(ros::Duration(5,0))){
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
                // ROS_INFO("STATE : %s", mc.getState().toString().c_str());   
                move_base_msgs::MoveBaseGoal goal_;
                goal_.target_pose.pose.position = p;
                goal_.target_pose.pose.orientation.w = 1.;
                goal_.target_pose.header.frame_id = "map";
                goal_.target_pose.header.stamp = ros::Time::now();
                mc.sendGoal(goal_);
                // ROS_INFO("send Goal");

                mc.waitForResult(ros::Duration(6,0));
            }
        }
        
        void publish_frontier(){
            //INITIALIZE FRONTIER, FRONTIER GROUPS, 
            visualization_msgs::Marker points, cluster_point, center_point;
            points.header.frame_id = cluster_point.header.frame_id = center_point.header.frame_id = "map";
            points.header.stamp =  cluster_point.header.stamp = center_point.header.stamp= ros::Time::now();
            cluster_point.ns = center_point.ns ="points_and_lines";
            points.action = cluster_point.action = center_point.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = cluster_point.pose.orientation.w = center_point.pose.orientation.w =1.0;

            points.id = 0;
            cluster_point.id = 0;
            center_point.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            cluster_point.type = visualization_msgs::Marker::POINTS;
            center_point.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.05;
            points.scale.y = 0.05;
            cluster_point.scale.x = 0.2;
            cluster_point.scale.y = 0.2;
            center_point.scale.x = 0.2;
            center_point.scale.y = 0.2;
            points.color.r = 1.0;
            points.color.a = 2.0;
            cluster_point.color.b = 1.0;
            cluster_point.color.a = 3.0;
            center_point.color.r = 1.0;
            center_point.color.g = 1.0;
            center_point.color.b = 0.0;
            center_point.color.a = 1.0;
            
           
            for(int i=0; i<frontier.size();i++){
                geometry_msgs::Point p;
                int t_x = frontier[i] % map->info.width;
                int t_y = frontier[i] / map->info.width;
                p.x = (t_x*map->info.resolution) + map->info.origin.position.x + map->info.resolution /2;
                p.y = (t_y*map->info.resolution) + map->info.origin.position.y + map->info.resolution /2;
                p.z = 0.3;
                points.points.push_back(p);
            }
            total_frontier_num = points.points.size();

            clock_t start, end;
            start = clock();
            for(int j=0; j<group_id; j++){
                int num_frontier = clustered[j].frontier_index.size();
                if(num_frontier>20){
                    // geometry_msgs::Point p1;
                    geometry_msgs::Point p2;
                    // p1.z=0.1;
                    p2.z=0.5;
                    // for(int i=0;i<clustered[j].frontier_index.size();i++){
                    //     int t_x = clustered[j].frontier_index[i] % map->info.width;
                    //     int t_y = clustered[j].frontier_index[i] / map->info.width;
                    //     p1.x = (t_x*map->info.resolution) + map->info.origin.position.x + map->info.resolution /2;
                    //     p1.y = (t_y*map->info.resolution) + map->info.origin.position.y + map->info.resolution /2;
                    //     cluster_point.points.push_back(p1);
                    // }
                    int t_x = clustered[j].frontier_index[num_frontier/2] % map->info.width;
                    int t_y = clustered[j].frontier_index[num_frontier/2] / map->info.width;
                    p2.x = (t_x*map->info.resolution) + map->info.origin.position.x + map->info.resolution /2;
                    p2.y = (t_y*map->info.resolution) + map->info.origin.position.y + map->info.resolution /2;
                    center_point.points.push_back(p2);
                    valid_cluster.push_back(j);
                    //Calculate number factor
                    clustered[j].num_factor = (double)clustered[j].frontier_index.size() / (double)total_frontier_num;
                }
            }
            end = clock();
            cal_size_time = (double)(end - start)/CLOCKS_PER_SEC*1000;

            frontier_pub_.publish(points);
            // cluster_pub_.publish(cluster_point);
            center_frontier_pub_.publish(center_point);
        }

        

        //utils
        int gridTomap(int x, int y, int width){
            return y * width + x;
        }

        void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono_img){
            if(mono_img.rows != float_img.rows || mono_img.cols != float_img.cols){
                mono_img = cv::Mat(float_img.size(), CV_8UC1);
            }
            cv::convertScaleAbs(float_img, mono_img, 1.0, 0.0);
        }

        void clear_vectors(){
            frontier.clear();
            visited.clear();
            frontier_map.clear();
            valid_cluster.clear();
            regions.clear();
            for(int i=0; i<group_id;i++){
                clustered[i].frontier_index.clear();
            }
            detect_time=cluster_time=cal_size_time=cal_dist_time=cal_region_time=cal_region_time_=total=0;
        }

        ~Frontiers(){
            clear_vectors();
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "autonomous exploration");
    Frontiers khs("map");
    ros::spin();

    return 0;
}