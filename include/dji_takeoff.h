#ifndef DJI_TAKEOFF_H
#define DJI_TAKEOFF_H

#include <ros/ros.h>
#include <djisdk/dji_sdk.h>

bool obtainControl();

bool monitoredTakeoff();

bool takeoffLand(int task);

#endif //DJI_TAKEOFF_H