#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptr_sor_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc,*ptr_sor_filtered);

    int num_neigbor_points = 50.0;
    double std_multiplier = 50.0;

    // *ptr_sor_filtered = temp_cloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (ptr_sor_filtered);
    sor.setMeanK (num_neigbor_points);
    sor.setStddevMulThresh (std_multiplier);
    sor.filter(*ptr_sor_filtered);

    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    pcl::toPCLPointCloud2(*ptr_sor_filtered, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);

    // Publish the data
    pub.publish(ros_output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "pcl_voxel");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/zedm/zed_node/point_cloud/cloud_registered", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered", 1);

    // Spin
    ros::spin ();
}