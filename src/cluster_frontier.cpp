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

    //subscriber handler
    ros::Subscriber map_sub_;

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

    //check computation times
    double detect_time=0;
    double cluster_time=0;
    double cal_size_time=0;
    double cal_dist_time=0;
    double cal_region_time=0;
    double total=0;
    int call_cn=0;

    public:
        Frontiers(const std::string& mapname): it_(n)
        {
            //subscribe
            map_sub_ = n.subscribe("map", 1, &Frontiers::mapCallback, this);
            
            //publisher
            frontier_pub_ =  n.advertise<visualization_msgs::Marker>("/frontiers", 1);
            center_frontier_pub_ =  n.advertise<visualization_msgs::Marker>("/center_frontiers", 1);
            // cluster_pub_ =  n.advertise<visualization_msgs::Marker>("/grouped_frontiers", 1);
            //service

        }

        void mapCallback(const nav_msgs::OccupancyGridConstPtr& map_)
        {
            map = map_;
            call_cn++;
            detect_frontier();
            cluster_frontier();
            publish_frontier();
            print_status();
            clear_vectors();
        }

        void print_status(){
            total = detect_time+cluster_time+cal_size_time+cal_dist_time+cal_region_time;
            printf("STATUS------------------------------------\n");
            ROS_INFO("[#%d]%d x %d map @ %.3f m/pix",call_cn, map->info.width, map->info.height, map->info.resolution);
            printf("detect %.1f ms\tcluster %.1f ms\tsize %.1f ms\n",detect_time,cluster_time,cal_size_time);
            printf("dist %.1f ms\tregion %.1f ms\t\e[1mtotal %.1f ms\e[0m\n",cal_dist_time,cal_region_time,total);
            printf("------------------------------------\n");
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

        void clear_vectors(){
            frontier.clear();
            visited.clear();
            frontier_map.clear();
            valid_cluster.clear();
            for(int i=0; i<group_id;i++){
                clustered[i].frontier_index.clear();
            }
        }
            
};

int main(int argc, char **argv){
    ros::init(argc, argv, "cluster_frontier");
    Frontiers khs("map");
    ros::spin();

    return 0;
}