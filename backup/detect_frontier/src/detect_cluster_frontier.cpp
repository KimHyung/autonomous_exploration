#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "image_transport/image_transport.h"
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"

#include <cmath>
#include <vector>
#include <sstream>
#include <queue>
#include <algorithm>

using namespace std;


visualization_msgs::Marker points;


void mapCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg)
{
    ros::NodeHandle n;

    


    if(msg->boxes.size()!=0){
        for(int i=0; i<msg->boxes.size(); i++){
            points.header.frame_id = "map";
            points.header.stamp = ros::Time::now();
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;
            points.id = 0;
            points.type = visualization_msgs::Marker::POINTS;
            points.scale.x = 0.1;
            points.scale.y = 0.1;
            points.color.r = 1.0;
            points.color.a = 2.0;
            if(msg->boxes[i].pose.position.x==0&&msg->boxes[i].pose.position.y==0&&msg->boxes[i].pose.position.z==0){
                continue;
            }
            else{
                cout<<"centerpoint:"<<msg->boxes[i].pose.position.x<<","<<msg->boxes[i].pose.position.y<<","<<msg->boxes[i].pose.position.z<<endl;
                cout<<"dimensions:"<<msg->boxes[i].dimensions.x<<','<<msg->boxes[i].dimensions.y<<','<<msg->boxes[i].dimensions.z<<endl;
                
                // int t_y = frontier[i] / map->info.width;
                // p.x = msg->boxes[i].pose.position.x;
                // p.y = msg->boxes[i].pose.position.y;
                // p.z = msg->boxes[i].pose.position.z;
                // points.points.push_back(p);
            }
        }
        // frontier_pub_.publish(points);
    }
    ros::spinOnce();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_cluster_frontier");
    ros::NodeHandle n;

    ros::Subscriber map_sub = n.subscribe("cluster_decomposer/boxes", 2000, mapCallback);
    // ros::Publisher frontier_pub_ = n.advertise<visualization_msgs::Marker>("/test", 1);
    // if(points.points.size()!=0){
    //     cout<<"IN"<<endl;
    //     frontier_pub_.publish(points);
    // }

    ros::spin();

    return 0;
}
