/*
 * The MIT License
 *
 * Copyright (c) 2019 Giovanni di Dio Bruno https://gbr1.github.io
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <upboard_ros/Led.h>
#include <upboard_ros/Leds.h>

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

float wr, wl;


void callbackMotor(const geometry_msgs::Twist& msg){
    wr=(msg.linear.x+msg.angular.z*0.057)/0.35;
    wl=(msg.linear.x-msg.angular.z*0.057)/0.35;
    vx=msg.linear.x;
    vth=msg.angular.z;   
}



int main(int argc, char** argv){
    ros::init(argc, argv, "sengi_sim");
    ros::NodeHandle nh;
    ros::Subscriber emulateDrive = nh.subscribe("/erwhi_velocity_controller/cmd_vel",1,callbackMotor);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    //ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_state", 50);
    ros::Publisher led_pub = nh.advertise<upboard_ros::Leds>("/upboard/leds",10);
    tf::TransformBroadcaster odom_broadcaster;
    upboard_ros::Led led_msg;
    upboard_ros::Leds leds_msg;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(50.0);
    while(nh.ok()){
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        //publish the message
        odom_pub.publish(odom);

        /*
        sensor_msgs::JointState js;
        js.header.frame_id="base_link";
        js.header.stamp=ros::Time::now();
        js.name[0]="left_joint";
        js.position[0]=wl/dt;
        js.velocity[0]=wl;
        js.name[1]="right_joint";
        js.position[1]=wr/dt;
        js.velocity[1]=wr;

        joint_pub.publish(js);
        */

        //publish leds update
        //right wheel blue forward, yellow backward
        leds_msg.header.stamp=ros::Time::now();
        leds_msg.header.frame_id="base_link";
        if (wr>0){
            led_msg.led=led_msg.BLUE;
            led_msg.value=1;
            leds_msg.leds.push_back(led_msg);
            led_msg.led=led_msg.YELLOW;
            led_msg.value=0;
            leds_msg.leds.push_back(led_msg);
        }
        else {
            if (wr<0){
                led_msg.led=led_msg.BLUE;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
                led_msg.led=led_msg.YELLOW;
                led_msg.value=1;
                leds_msg.leds.push_back(led_msg);
            }
            else{
                led_msg.led=led_msg.BLUE;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
                led_msg.led=led_msg.YELLOW;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
            }
        }
        if (wl>0){
            led_msg.led=led_msg.GREEN;
            led_msg.value=1;
            leds_msg.leds.push_back(led_msg);
            led_msg.led=led_msg.RED;
            led_msg.value=0;
            leds_msg.leds.push_back(led_msg);
        }
        else {
            if (wl<0){
                led_msg.led=led_msg.GREEN;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
                led_msg.led=led_msg.RED;
                led_msg.value=1;
                leds_msg.leds.push_back(led_msg);
            }
            else{
                led_msg.led=led_msg.GREEN;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
                led_msg.led=led_msg.RED;
                led_msg.value=0;
                leds_msg.leds.push_back(led_msg);
            }
        }
        led_pub.publish(leds_msg);
        leds_msg.leds.clear();

        last_time = current_time;
        r.sleep();
    }
}