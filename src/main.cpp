//
// Created by phil on 24/01/18.
//

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

#include <iostream>

ros::Publisher pub;
bool airborne;

// Forces callback function
void forcesCb(ConstContactsPtr &_msg){
    geometry_msgs::Vector3 msgForce;
    // What to do when callback
    for (int i = 0; i < _msg->contact_size(); ++i) {
        msgForce.x = _msg->contact(i).normal().Get(0).x();
        msgForce.y = _msg->contact(i).normal().Get(0).y();
        msgForce.z = _msg->contact(i).normal().Get(0).z();
        pub.publish(msgForce);
        gazebo::common::Time::MSleep(20);
    }
    if ( _msg->contact_size()== 0){
        msgForce.x = 0;
        msgForce.y = 0;
        msgForce.z = 0;
        pub.publish(msgForce);
    }

}

// Position callback function
void positionCb(const nav_msgs::Odometry::ConstPtr& msg2){
    if (msg2->pose.pose.position.z > 0.3) {
        airborne = true;
    } else {
        airborne = false;
    }
}

int main(int _argc, char **_argv){
    // Set variables
    airborne = false;

    // Load Gazebo & ROS
    gazebo::client::setup(_argc, _argv);
    ros::init(_argc, _argv, "force_measure");

    // Create Gazebo node and init
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Create ROS node and init
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::Vector3>("forces", 1000);

    // Listen to Gazebo contacts topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", forcesCb);

    // Listen to ROS for position
    ros::Subscriber sub2 = n.subscribe("ground_truth/state", 1000, positionCb);

    // Busy wait loop...replace with your own code as needed.
    // Busy wait loop...replace with your own code as needed.
    while (true)
    {
        gazebo::common::Time::MSleep(20);

        // Spin ROS (needed for publisher) // (nope its actually for subscribers-calling callbacks ;-) )
        ros::spinOnce();


    // Mayke sure to shut everything down.

    }
    gazebo::client::shutdown();
}