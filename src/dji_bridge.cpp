#include "dji_bridge.h"
#include <dji_sdk/dji_sdk.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include "uav_msgs/uav_pose.h"
#include <boost/thread.hpp>
#include <math.h>
using namespace DJI::OSDK;
void NED_ENU(double *in, double *out) {
    out[0] = in[1];
    out[1] = in[0];
    out[2] = -in[2];
}

dji_bridge *globaldji_bridge;
class dji_bridge_priv {
  public:
    int argc;
    char* *argv;
    ros::Time startTime;
    ros::Publisher imu_pub, uavpose_corrected_pub, uavpose_pub, ctrlPosYawPub;
    ros::NodeHandle *nh = NULL;
    boost::mutex offsetMutex, localPosMutex, localVelMutex;
    ros::ServiceClient set_local_pos_reference;
    ros::ServiceClient sdk_ctrl_authority_service;
    ros::ServiceClient drone_task_service;
    ros::ServiceClient query_version_service;
    const tf::Matrix3x3 R_ENU2NED = tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1), R_FRD2FLU = tf::Matrix3x3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    double control[4], vel[3], poi[3], offset[3], localPos[3], localVel[3];
    std::string nameSpace;
    uint32_t sequence;
    uint8_t flight_status = 255;
    uint8_t display_mode  = 255;


    void commandCallback(const uav_msgs::uav_pose::ConstPtr & msg) {
      double offset_[3], control_[4], vel_[3], poi_[3], localPos_[3];
      uav_msgs::uav_pose uavpose;
      offsetMutex.lock();
      offset_[0] = offset[0];
      offset_[1] = offset[1];
      offset_[2] = offset[2];
      offsetMutex.unlock();
      control_[0] = msg->position.x - offset_[0];
      control_[1] = msg->position.y - offset_[1];
      control_[2] = msg->position.z - offset_[2];
      control_[3] = 0;
      NED_ENU(control_, control);
      vel_[0] = msg->velocity.x;
      vel_[1] = msg->velocity.y;
      vel_[2] = msg->velocity.z;
      NED_ENU(vel_, vel);
      poi_[0] = msg->POI.x - offset_[0];
      poi_[1] = msg->POI.y - offset_[1];
      poi_[2] = msg->POI.z - offset_[2];
      NED_ENU(poi_, poi);
      localPosMutex.lock();
      localPos_[0] = localPos[0];
      localPos_[1] = localPos[1];
      localPos_[2] = localPos[2];
      localPosMutex.unlock();

      double targetAng = RAD2DEG(atan2f(control[1]-localPos_[0], control[0]-localPos_[1]));
      targetAng = targetAng > 0?targetAng:targetAng+360.0;
      double poiAng = RAD2DEG(atan2f(poi[1]-localPos_[0], poi[0]-localPos_[1]));
      poiAng = poiAng > 0?poiAng:poiAng+360.0;
      double yaw = poiAng - targetAng;
      if (yaw > 180.0) yaw = yaw - 360.0;
      else if (yaw < -180.0) yaw = yaw + 360.0; 

      sensor_msgs::Joy controlPosYaw;
      controlPosYaw.axes.push_back(control[0]-localPos_[0]);
      controlPosYaw.axes.push_back(control[1]-localPos_[1]);
      controlPosYaw.axes.push_back(control[2]-localPos_[2]);
      controlPosYaw.axes.push_back(yaw);
      ctrlPosYawPub.publish(controlPosYaw);
    }
    void offsetCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
      double offset_[3];
      offsetMutex.lock();
      offset_[0] = msg->pose.pose.position.x;
      offset_[1] = msg->pose.pose.position.y;
      offset_[2] = msg->pose.pose.position.z;
      NED_ENU(offset_, offset);
      offsetMutex.unlock();
    }
    void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
      double localPos_[3];
      localPosMutex.lock();
      localPos_[0] = msg->point.x;
      localPos_[1] = msg->point.y;
      localPos_[2] = msg->point.z;
      localPosMutex.unlock();
      NED_ENU(localPos_, localPos);
    }
    // void poseCallback(const geometry_msgs::TransformStamped::ConstPtr & msg) {
    //     return;
    // }
    void local_vel_callback(const geometry_msgs::Vector3Stamped::ConstPtr & msg) {
      localVelMutex.lock();
      localVel[1] = msg->vector.x;
      localVel[0] = msg->vector.y;
      localVel[2] = -msg->vector.z;
      localVelMutex.unlock();
    }
    void local_imu_callback(const sensor_msgs::Imu::ConstPtr & msg) {
      uav_msgs::uav_pose uavpose;
      boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::universal_time() - boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1));

      uavpose.header.frame_id = "world";
      uavpose.header.seq = sequence++;
      uavpose.header.stamp.sec = diff.total_seconds();
      uavpose.header.stamp.nsec = 1000 * (diff.total_microseconds() % 1000000);


      tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(uavpose.orientation.x, uavpose.orientation.y, uavpose.orientation.z, uavpose.orientation.w));
      tf::Matrix3x3 R_FRD2NED = R_ENU2NED * R_FLU2ENU * R_FRD2FLU;
      tf::Quaternion q_FRD2NED;
      R_FRD2NED.getRotation(q_FRD2NED);
      uavpose.orientation.w = q_FRD2NED.getW();
      uavpose.orientation.x = q_FRD2NED.getX();
      uavpose.orientation.y = q_FRD2NED.getY();
      uavpose.orientation.z = q_FRD2NED.getZ();
      
      uavpose.flightmode    = flight_status;
      uavpose.angVelocity.x = msg->angular_velocity.x;
      uavpose.angVelocity.y = msg->angular_velocity.y;
      uavpose.angVelocity.z = msg->angular_velocity.z;
      localPosMutex.lock();
      localVelMutex.lock();
      uavpose.position.x = localPos[0];
      uavpose.position.y = localPos[1];
      uavpose.position.z = localPos[2];
      uavpose.velocity.x = localVel[0];
      uavpose.velocity.y = localVel[1];
      uavpose.velocity.z = localVel[2];
      localVelMutex.unlock();
      localPosMutex.unlock();
      uavpose_pub.publish(uavpose);

      offsetMutex.lock();
      uavpose.position.x += offset[1];
      uavpose.position.y += offset[0];
      uavpose.position.z -= offset[2];
      offsetMutex.unlock();
      uavpose_corrected_pub.publish(uavpose);
    }
    void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg){
      flight_status = msg->data;
    }

    void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg){
      display_mode = msg->data;
    }
    bool obtain_control(){
      dji_sdk::SDKControlAuthority authority;
      authority.request.control_enable=1;
      sdk_ctrl_authority_service.call(authority);

      if(!authority.response.result)
      {
        ROS_ERROR("obtain control failed!");
        return false;
      }

      return true;
    }

    bool monitoredTakeoff(){
      ros::Time start_time = ros::Time::now();

      if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
        return false;
      }

      ros::Duration(0.01).sleep();
      ros::spinOnce();

      // Step 1.1: Spin the motor
      while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
            display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
            ros::Time::now() - start_time < ros::Duration(5)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }

      if(ros::Time::now() - start_time > ros::Duration(5)) {
        ROS_ERROR("Takeoff failed. Motors are not spinnning.");
        return false;
      }
      else {
        start_time = ros::Time::now();
        ROS_INFO("Motor Spinning ...");
        ros::spinOnce();
      }


      // Step 1.2: Get in to the air
      while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
              (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
              ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }

      if(ros::Time::now() - start_time > ros::Duration(20)) {
        ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
        return false;
      }
      else {
        start_time = ros::Time::now();
        ROS_INFO("Ascending...");
        ros::spinOnce();
      }

      // Final check: Finished takeoff
      while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
              ros::Time::now() - start_time < ros::Duration(20)) {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
      }

      if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
      {
        ROS_INFO("Successful takeoff!");
        start_time = ros::Time::now();
      }
      else
      {
        ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
        return false;
      }

      return true;
    }

    bool takeoff_land(int task){
      dji_sdk::DroneTaskControl droneTaskControl;

      droneTaskControl.request.task = task;

      drone_task_service.call(droneTaskControl);

      if(!droneTaskControl.response.result)
      {
        ROS_ERROR("takeoff_land fail");
        return false;
      }

      return true;
    }

    bool set_local_position(){
      dji_sdk::SetLocalPosRef localPosReferenceSetter;
      set_local_pos_reference.call(localPosReferenceSetter);
      return localPosReferenceSetter.response.result;
    }
 
};
dji_bridge::dji_bridge(int argc, char* *argv) {
    globaldji_bridge = this;
    instance = new dji_bridge_priv();
    instance->argc = argc;
    instance->argv = argv;
    instance->sequence = 0;
    instance->startTime = ros::Time::now();
}

dji_bridge::~dji_bridge() {
    if (instance->nh) 
        delete instance->nh;
    delete instance;
}

int dji_bridge::run() {
    ros::init(instance->argc, instance->argv, "dji_node");
    
    instance->nh = new ros::NodeHandle();

    if (instance->argc < 2) {
        printf("Usage: %s <namespace>\n", instance->argv[0]);
        return -1;
    }

    instance->nameSpace = std::string(instance->argv[1]);
    
    instance->uavpose_pub   = instance->nh->advertise<uav_msgs::uav_pose>(instance->nameSpace + "/pose/raw", 10);
    instance->uavpose_corrected_pub = instance->nh->advertise<uav_msgs::uav_pose>(instance->nameSpace + "/pose", 10);
    instance->ctrlPosYawPub = instance->nh->advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);
    // ros::Subscriber subscriber1 = instance->nh->subscribe("vicon/octocopter/frame", 10, &writethread_priv::poseCallback, this);
    ros::Subscriber subscriber2 = instance->nh->subscribe(instance->nameSpace + "/command", 10, &dji_bridge_priv::commandCallback, instance);
    ros::Subscriber subscriber3 = instance->nh->subscribe(instance->nameSpace + "/offset", 10, &dji_bridge_priv::offsetCallback, instance);
    ros::Subscriber subscriber4 = instance->nh->subscribe("dji_sdk/local_position", 10, &dji_bridge_priv::local_position_callback, instance);
    ros::Subscriber subscriber5 = instance->nh->subscribe("dji_sdk/imu", 10, &dji_bridge_priv::local_imu_callback, instance);
    ros::Subscriber subscriber6 = instance->nh->subscribe("dji_sdk/velocity", 10, &dji_bridge_priv::local_vel_callback, instance);
    ros::Subscriber flightStatusSub = instance->nh->subscribe("dji_sdk/flight_status", 10, &dji_bridge_priv::flight_status_callback, instance);
    ros::Subscriber displayModeSub = instance->nh->subscribe("dji_sdk/display_mode", 10, &dji_bridge_priv::display_mode_callback, instance);


    instance->sdk_ctrl_authority_service = instance->nh->serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
    instance->drone_task_service         = instance->nh->serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    instance->query_version_service      = instance->nh->serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    instance->set_local_pos_reference    = instance->nh->serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

    bool obtain_control_result = instance->obtain_control();
    bool takeoff_result;
    if (!instance->set_local_position()) // We need this for height
    {
        ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
        return 1;
    }
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = instance->monitoredTakeoff();

    ros::AsyncSpinner spinner(6);
    spinner.start();
    ros::waitForShutdown();
}


