/*
 * @Description: ins数据转换的node文件
 * @Author: WYK & WH & YPJ
 * @Date: 2022-03-18 10:17.03
 */

#include <iostream>
#include <sstream>
#include <string>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include "nmea_msgs/Sentence.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include "localization_msgs/gnss.h"

const int covariance_enable = 1;

class MsgCoverter{
public:
    MsgCoverter(ros::NodeHandle &nh, std::string rec_topic_name,  
                std::string gps_topic_name,
                std::string vel_topic_name,
                std::string imu_topic_name,
                std::string gnss_topic_name);

private:
    void msg_callback(const nmea_msgs::Sentence::ConstPtr &icv_location_ptr);
    inline double angle_normalization(const double angle);

    float D2R = 3.1415926 / 180.0;
    float R2D = 180.0 / 3.1415926;

    ros::Subscriber subscriber_;
    ros::Publisher gnss_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher imu_pub_;
    ros::NodeHandle nh_;
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "converter_node");
  ros::NodeHandle nh;

  std::string rec_ins_topic;
  
  std::string gps_pub_topic;
  std::string vel_pub_topic;
  std::string imu_pub_topic;
  std::string gnss_pub_topic;
  std::cout<<"Reading parameters..."<<std::endl;
  nh.param<std::string>("rec_topic", rec_ins_topic, "/nmea_sentence"); 
  nh.param<std::string>("gnss_pub_topic", gnss_pub_topic, "/gnss_data"); 
  nh.param<std::string>("gps_pub_topic", gps_pub_topic, "/gps/fix");
  nh.param<std::string>("vel_pub_topic", vel_pub_topic, "/gps/vel"); 
  nh.param<std::string>("imu_pub_topic", imu_pub_topic, "/imu_raw"); 
  MsgCoverter converter(nh, rec_ins_topic, gps_pub_topic, vel_pub_topic, imu_pub_topic, gnss_pub_topic);
  std::cout<<"Start to coverter "<<rec_ins_topic<<" to "<<gnss_pub_topic<<std::endl;
  ros::spin();

  return 0;
}

MsgCoverter::MsgCoverter(ros::NodeHandle &nh, std::string rec_topic_name,
                         std::string gps_topic_name,
                         std::string vel_topic_name,
                         std::string imu_topic_name,
                         std::string gnss_topic_name)
:nh_(nh)
{
    subscriber_ = nh_.subscribe(rec_topic_name, 1, &MsgCoverter::msg_callback, this);
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(gps_topic_name, 1);
    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(vel_topic_name, 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_name, 1);
    gnss_pub_ = nh_.advertise<localization_msgs::gnss>(gnss_topic_name, 1);

}

void MsgCoverter::msg_callback(const nmea_msgs::Sentence::ConstPtr &icv_location_ptr)
{
  // 定义发布的数据
  sensor_msgs::NavSatFix navsatfix_msg;
  sensor_msgs::Imu imu_msg;
  geometry_msgs::TwistStamped vel_msg;
  localization_msgs::gnss gnss_msg;

  navsatfix_msg.header = icv_location_ptr->header;
  navsatfix_msg.header.frame_id = "imu_link";
  imu_msg.header = icv_location_ptr->header;
  imu_msg.header.frame_id = "imu_link";
  vel_msg.header = icv_location_ptr->header;
  vel_msg.header.frame_id = "imu_link";

  std::string sentence = icv_location_ptr->sentence;
  
  std::istringstream ss(sentence);
  std::string token;
  std::vector<std::string> datas;
  while(std::getline(ss, token, ',')) {
    datas.push_back(token);
  }

  int GPSWeek = std::stoi(datas[1]);
  double GPSTIme = std::stod(datas[2]);
  double roll = std::stod(datas[5]);
  double pitch = std::stod(datas[4]); 
  double yaw = angle_normalization(-std::stod(datas[3]));

  double gyro_x = std::stod(datas[6])*D2R;
  double gyro_y = std::stod(datas[7])*D2R;
  double gyro_z = std::stod(datas[8])*D2R;

  double acc_x = std::stod(datas[9]);
  double acc_y = std::stod(datas[10]);
  double acc_z = std::stod(datas[11]);

  double Lat = std::stod(datas[12]);
  double Lon = std::stod(datas[13]);
  double Alt = std::stod(datas[14]);

  double V_E = std::stod(datas[15]);
  double V_N = std::stod(datas[16]);
  double V_U = std::stod(datas[17]);
  double V   = std::stod(datas[18]);

  int NSV1 = std::stoi(datas[19]);
  int NSV2 = std::stoi(datas[20]);
  int status = std::stoi(datas[21]);
  status = (status== 42? 4:1);


  // gps数据解析
  // double curr_time = 7*86400*GPSWeek + GPSTIme + 315964800 -18;    //用于gps授时，否则使用电脑网络时间
  // navsatfix_msg.header.stamp = ros::Time(curr_time);
  navsatfix_msg.status.status = status;
  navsatfix_msg.latitude = Lat;
  navsatfix_msg.longitude = Lon;
  navsatfix_msg.altitude = Alt;
  // navsatfix_msg.altitude = 354.0;

  // imu数据解析
  //Z->Y->X,内旋
  tf::Quaternion q;
  q.setRPY(pitch * D2R,roll * D2R, yaw * D2R);
  imu_msg.orientation.x = q.x();
  imu_msg.orientation.y = q.y();
  imu_msg.orientation.z = q.z();
  imu_msg.orientation.w = q.w();

  imu_msg.angular_velocity.x = gyro_x;  //degree
  imu_msg.angular_velocity.y = gyro_y;
  imu_msg.angular_velocity.z = gyro_z;

  imu_msg.linear_acceleration.x = acc_x;
  imu_msg.linear_acceleration.y = acc_y;
  imu_msg.linear_acceleration.z = acc_z;

  if(covariance_enable == 1 )
  {
      for(int i = 0;i<9;i++)
    {
      imu_msg.orientation_covariance[i] = 0;
      imu_msg.angular_velocity_covariance[i] = 0;
      imu_msg.linear_acceleration_covariance[i] = 0;
    }
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.angular_velocity_covariance[8] = 0.01;
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;
  }
  
  // gnss数据解析
  gnss_msg.gps_data = navsatfix_msg;
  gnss_msg.imu_data = imu_msg;

  // 速度数据解析
  vel_msg.twist.linear.x = V_E;
  vel_msg.twist.linear.y = V_N;
  vel_msg.twist.linear.z = V_U;

  
  gnss_pub_.publish(gnss_msg);
  gps_pub_.publish(navsatfix_msg);
  vel_pub_.publish(vel_msg);
  imu_pub_.publish(imu_msg);
}

inline double MsgCoverter::angle_normalization(const double angle)
{
    double angle_norm = angle;
    while (angle_norm > 180)
    {
        angle_norm -= 360;
    }
    while (angle_norm < -180)
    {
        angle_norm += 360;
    }
    return angle_norm;
}
