#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/PointCloud2.h"

#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
#include "pcl_conversions/pcl_conversions.h"
//#include <string_view>


tf2_ros::Buffer* tf_buffer_ptr;
tf2_ros::TransformListener* tf_listener_ptr;

//tf::TransformListener* listener_ptr;
ros::Publisher* pcl_pub_ptr;

std::string goal_frame;
std::string fixed_frame;


struct data_cb
{
    sensor_msgs::PointCloud2 pcl;
    bool is_new = false;
};

std::map<std::string, data_cb> clouds_from_cb;






void combine_and_publish()
{
    pcl::PCLPointCloud2 combined_pcl;

    ros::Time time_now = ros::Time::now();

    //transform all pcls to goal frame and add them (at current time??)
    for(auto& data : clouds_from_cb)
    {
        
        geometry_msgs::TransformStamped transform_stamped;
        sensor_msgs::PointCloud2 transformed_cloud;
        
        data.second.is_new = false;

        // TODO: add a "wait for transform" and use the current time?
        try
        {
            transform_stamped = tf_buffer_ptr->lookupTransform(goal_frame, ros::Time(0), data.second.pcl.header.frame_id, data.second.pcl.header.stamp, fixed_frame);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            //ros::Duration(1.0).sleep();
            continue;
        }
        tf2::doTransform(data.second.pcl, transformed_cloud, transform_stamped);


        // convert to pcl::... and add to the combined cloud
        pcl::PCLPointCloud2 transformed_pcl;
        pcl_conversions::toPCL(transformed_cloud, transformed_pcl);        
        combined_pcl += transformed_pcl;

        

        /*
        if(!listener_ptr->waitForTransform(
                    data.second.pcl.header.frame_id,
                    goal_frame,
                    time_now,
                    ros::Duration(1.0)))
        {
            std::cout << "Could not find tf...\n";
            return;
        }        
        */
    }

    sensor_msgs::PointCloud2 combined_cloud;
    // convert back to sensor_msgs::PointCloud2
    pcl_conversions::fromPCL(combined_pcl, combined_cloud);


    pcl_pub_ptr->publish(combined_cloud);
    //std::cout << "PUBLISHED PCL!\n";
}



void scan_callback(const sensor_msgs::PointCloud2& msg)
{
    clouds_from_cb[msg.header.frame_id].pcl = msg;
    clouds_from_cb[msg.header.frame_id].is_new = true;




    return;





    // check if one msg from each frame in the map has been received
    bool all_received = true;
    for(const auto& data : clouds_from_cb)
    {

        //std::cout << "FRAMES::   " << data.first.c_str() << "\n";
        if(data.second.is_new != true)
        {
            //std::cout << "\ttrue? " << data.second.is_new << "\n";
            all_received = false;
            //return;
        }
    }

    if(all_received == true) // TODO: add a timeout as well, and if some frame is never updated remove it?
    {
        //std::cout << "COMBINE AND PUBLISH\n";
        // publish combined pointcloud with data from all frames that have been published to this node
        //combine_and_publish();
    }
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_adder");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // init
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    //tf::TransformListener listener_;
    ros::Publisher pcl_pub;
    //listener_ptr = &listener_;
    
    pcl_pub_ptr = &pcl_pub;
    tf_buffer_ptr = &tf_buffer;
    tf_listener_ptr = &tf_listener;



    double publish_rate;
    // ros param
    if (!nh_private.getParam ("fixed_frame", fixed_frame))
        fixed_frame = "world";
    if (!nh_private.getParam ("goal_frame", goal_frame))
        goal_frame = "base_link";
    if (!nh_private.getParam ("publish_rate", publish_rate))
        publish_rate = 25;

    ROS_INFO_STREAM("Using parameters:\n\tfixed frame: " << fixed_frame << "\n\tgoal_frame: " << goal_frame );





    ros::Subscriber sub_odom = nh.subscribe("clouds_in", 1000, scan_callback);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("combined_pcl", 1000);





    ros::Rate r = ros::Rate(publish_rate);
    while(nh.ok())
    {
        //std::cout << "LOOOOOOOOOP\n";
        ros::spinOnce();               // check for incoming messages
        combine_and_publish();
        r.sleep();
    }
}



