#include <frontier_exploration/frontier_search.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>

#include <frontier_exploration/costmap_tools.h>
#include <frontier_exploration/Frontier.h>
#include <nav_msgs/GetPlan.h>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               

namespace frontier_exploration{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

FrontierSearch::FrontierSearch(costmap_2d::Costmap2D &costmap) : costmap_(costmap) { }

std::list<Frontier> FrontierSearch::searchFrom(geometry_msgs::Point position){

    std::list<Frontier> frontier_list;
    //Sanity check that robot is inside costmap bounds before searching
    unsigned int mx,my;
    if (!costmap_.worldToMap(position.x,position.y,mx,my)){
        ROS_ERROR("Robot out of costmap bounds, cannot search for frontiers");
        return frontier_list;
    }

    //make sure map is consistent and locked for duration of search
    boost::unique_lock < costmap_2d::Costmap2D::mutex_t > lock(*(costmap_.getMutex()));

    map_ = costmap_.getCharMap();
    size_x_ = costmap_.getSizeInCellsX();
    size_y_ = costmap_.getSizeInCellsY();

    //initialize flag arrays to keep track of visited and frontier cells
    std::vector<bool> frontier_flag(size_x_ * size_y_, false);
    std::vector<bool> visited_flag(size_x_ * size_y_, false);

    //initialize breadth first search
    std::queue<unsigned int> bfs;

    //find closest clear cell to start search
    unsigned int clear, pos = costmap_.getIndex(mx,my);
    if(nearestCell(clear, pos, FREE_SPACE, costmap_)){
        bfs.push(clear);
    }else{
        bfs.push(pos);
        ROS_WARN("Could not find nearby clear cell to start search");
    }
    visited_flag[bfs.front()] = true;

    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();

        //iterate over 4-connected neighbourhood
        BOOST_FOREACH(unsigned nbr, nhood8(idx, costmap_)){
            //add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
            //FREE_SPACE == map_[nbr]_
            if(map_[nbr] <= map_[idx] && !visited_flag[nbr]){
                visited_flag[nbr] = true;
                bfs.push(nbr);
                //check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
            }else if(isNewFrontierCell(nbr, frontier_flag)){
                frontier_flag[nbr] = true;
                Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag, position);
                if(new_frontier.size > 1){
                    frontier_list.push_back(new_frontier);
                }
            }
        }
    }

    return frontier_list;

}

void chatterCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    ROS_WARN("HI!~!");
    std::cout<<msg<<std::endl;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell, unsigned int reference, std::vector<bool>& frontier_flag,geometry_msgs::Point position){

    
    //khs
    // boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    ros::NodeHandle n;
    ros::ServiceClient check_path = n.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");
    // ros::ServiceClient subpose = n.serviceClient<geometry_msgs::Pose>("robot_pose_publisher");
    // ros::Subscriber subpose = n.subscribe("robot_pose",1000,chatterCallback);
    // geometry_msgs::Pose khs;
    // if(subpose.call(khs)){
    //     ROS_ERROR("WAAAAAAAAAAAAAAAAAAAAAA");
    //     std::cout<< khs.position<<std::endl;
    // }
    
    // std::cout << khs.position <<std::endl; 
    nav_msgs::GetPlan srv_plan;
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;
    tf::Stamped<tf::Pose> robot_pose;
    tf::TransformListener tf_listener_;
    // explore_costmap_ros_ = boost::shared_ptr<costmap_2d::Costmap2DROS>(new costmap_2d::Costmap2DROS("explore_server", tf_listener_));
    //initialize frontier structure
    Frontier output;
    output.centroid.x = 0;
    output.centroid.y = 0;
    output.size = 1;
    output.min_distance = std::numeric_limits<double>::infinity();

    //record initial contact point for frontier
    unsigned int ix, iy;
    costmap_.indexToCells(initial_cell,ix,iy);
    costmap_.mapToWorld(ix,iy,output.initial.x,output.initial.y);

    //push initial gridcell onto queue
    std::queue<unsigned int> bfs;
    bfs.push(initial_cell);
    // explore_costmap_ros_->resetLayers();
    //cache reference position in world coords
    // explore_costmap_ros_->getRobotPose(robot_pose);
           
    unsigned int rx,ry;
    double reference_x, reference_y;
    costmap_.indexToCells(reference,rx,ry);
    costmap_.mapToWorld(rx,ry,reference_x,reference_y);
    // tf::poseStampedTFToMsg(robot_pose,srv_plan.request.start);
    // std::cout<<costmap_.getCost(rx,ry) <<std::endl;
    
    while(!bfs.empty()){
        unsigned int idx = bfs.front();
        bfs.pop();
        //try adding cells in 8-connected neighborhood to frontier
        BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
            //check if neighbour is a potential frontier cell
            if(isNewFrontierCell(nbr,frontier_flag)){
                //mark cell as frontier
                frontier_flag[nbr] = true;
                unsigned int mx,my;
                double wx,wy;
                costmap_.indexToCells(nbr,mx,my);
                costmap_.mapToWorld(mx,my,wx,wy);

                //update frontier size
                output.size++;

                //update centroid of frontier
                output.centroid.x += wx;
                output.centroid.y += wy;
                
                
                // ROS_INFO("Make plan: %d", (check_path.call(srv_plan) ? 1 : 0));
                // ROS_INFO("Plan size: %d", srv_plan.response.plan.poses.size());
    
                double distance = sqrt(pow((double(reference_x)-double(wx)),2.0) + pow((double(reference_y)-double(wy)),2.0));
                if(distance < output.min_distance && distance != 0){
                    // output.min_distance = distance;
                    output.middle.x = wx;
                    output.middle.y = wy;
                }        
                //add to queue for breadth first search
                bfs.push(nbr);
            }
        }
    }

    //average out frontier centroid
    output.centroid.x /= output.size;
    output.centroid.y /= output.size;
    goal.pose.position.x=output.centroid.x;
    goal.pose.position.y=output.centroid.y;
    srv_plan.request.start.pose.position.x = position.x;
    srv_plan.request.start.pose.position.y = position.y;
    srv_plan.request.goal = goal;
    srv_plan.request.goal.header.frame_id = "map";
    srv_plan.request.start.header.frame_id = "map";
    srv_plan.request.tolerance = 1.5;
    int tmp = srv_plan.response.plan.poses.size();
    if(tmp !=0){
        output.min_distance = tmp;
    }
    else{
        output.min_distance =  2,147,483,647;
    }
    
    
    
    // goal.pose.position.x=output.centroid.x;
    // goal.pose.position.y=output.centroid.y;
    // tf::poseStampedTFToMsg(robot_pose,srv_plan.request.start);
    // srv_plan.request.goal = goal;
    // srv_plan.request.goal.header.frame_id = "map";
    // srv_plan.request.start.header.frame_id = "map";
    // srv_plan.request.tolerance = 1.5;
    // ROS_INFO("Make plan: %d", (check_path.call(srv_plan) ? 1 : 0));
    // ROS_INFO("Plan size: %d", srv_plan.response.plan.poses.size());
    // if(check_path.call(srv_plan)){
    //     double distance = srv_plan.response.plan.poses.size();
    //     if(distance != 0){
    //         output.min_distance = distance;
    //         output.middle.x = output.centroid.x;
    //         output.middle.y = output.centroid.y;
    //     }        
    // }
    return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx, const std::vector<bool>& frontier_flag){

    //check that cell is unknown and not already marked as frontier
    if(map_[idx] != NO_INFORMATION || frontier_flag[idx]){
        return false;
    }

    //frontier cells should have at least one cell in 4-connected neighbourhood that is free
    BOOST_FOREACH(unsigned int nbr, nhood8(idx, costmap_)){
        if(map_[nbr] == FREE_SPACE){
            return true;
        }
    }

    return false;

}

}
