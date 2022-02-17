//
// Created by Hyungtae Lim on 6/23/21.
//

// For disable PCL complile lib, to use PointXYZILID
#define PCL_NO_PRECOMPILE
#include "patchwork/patchwork.hpp"
#include <cstdlib>


using PointType = pcl::PointXYZ;
using namespace std;

ros::Publisher PositivePublisher;
ros::Publisher NegativePublisher;

boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;

std::string output_filename;
std::string acc_filename, pcd_savepath;
string      algorithm;
string      input_topic_name;
string      output_topic_name;
string      mode;
string      seq;
bool        save_flag;

template<typename T>
pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
    pcl::PointCloud<T> cloudresult;
    //cout << "Received cloudmsg with fields: " << cloudmsg.fields[3] << endl;
    pcl::fromROSMsg(cloudmsg,cloudresult);
    //cout << "Received type name: " << typeid(cloudresult).name() << endl;
    //cout << "Converted pcl with fields: " << cloudresult << endl;
    return cloudresult;
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "base_link")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


void callbackNode(const sensor_msgs::PointCloud2 cloudmsg) 
{
    cout << "Operating patchwork..." << endl;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    static double time_taken;
    
    pcl::PointCloud<PointType> pc_curr = cloudmsg2cloud<PointType>(cloudmsg);
    PatchworkGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken);
    PositivePublisher.publish(cloud2msg(pc_ground,cloudmsg.header.frame_id));
    NegativePublisher.publish(cloud2msg(pc_non_ground,cloudmsg.header.frame_id));
    }
    
    


int main(int argc, char **argv) {
    ros::init(argc, argv, "Benchmark");
    ros::NodeHandle nh;
    nh.param<string>("/algorithm", algorithm, "patchwork");
    nh.param<string>("/ground_seg_input_topic", input_topic_name, "/sensing/top/lidar/points_raw");
    nh.param<string>("/ground_seg_output_topic", output_topic_name, "/sensing/lidar/no_ground/pointcloud");
    ros::Rate loop_rate(10);

    PatchworkGroundSeg.reset(new PatchWork<PointType>(&nh));

    PositivePublisher     = nh.advertise<sensor_msgs::PointCloud2>("/ground_segmentation/ground_cloud", 100);
    NegativePublisher     = nh.advertise<sensor_msgs::PointCloud2>(output_topic_name, 100);
    
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(input_topic_name, 10, callbackNode);
    
    ros::spin();
    
    return 0;
}
