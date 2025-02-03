#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sstream>
#include <protocol.hpp>
#include <crc16.hpp>
#include <string>
#include<sys/shm.h>
#include <fstream>
class Communication{

private:
    double pitch,roll,yaw;
    int shmid = shmget((key_t)1234,sizeof(Protocol_DownPackage_t),0666|IPC_CREAT);
    void * shm = NULL;
    Protocol_DownPackage_t *stu = NULL;
    serial::Serial ser;
    ros::NodeHandle n1;
    Protocol_UpPackageMCU_t mcu_data;
    Protocol_UpDataReferee_t ref_data;
    

public:
    Communication();
    int serial_init(std::string name, int baud);
    void vel_callback(const geometry_msgs::Twist &cmd_vel);
    void Odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void receive_MCU();


};