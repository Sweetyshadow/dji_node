#ifndef DJI_BRIDGE_H
#define DJI_BRIDGE_H

#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * ((180.0) / (C_PI)))

class dji_bridge_priv;

class dji_bridge {
    public:
        int run();
            



        dji_bridge(int argc, char* *argv);
        ~dji_bridge();
    private:
        dji_bridge_priv *instance;
};

#endif // DJI_BRIDGE_H