#ifndef BWENVSENSORS_H
#define BWENVSENSORS_H

#include "bw_env_sensors/AsyncSerial.h"

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include "bw_env_sensors/EnvSensors.h"
#include "std_msgs/Int32.h"

#define PI 3.14159265

namespace bw_env_sensors
{
typedef struct {
  int temperature_org;
  int rh_org;
  int smoke_org;
  int pm1_0_org;
  int pm2_5_org;
  int pm10_org;
  int lel_org;
  int noise_org;

  float temperature;   //Centigrade
  float rh;            //relative humidity %RH
  float smoke;         //ppm
  float pm1_0;         //ug/m^3
  float pm2_5;         //ug/m^3
  float pm10;          //ug/m^3
  float lel;           //ppm
  float noise;         //db
}SENSOR_STATUS;

class StatusPublisher
{
public:
    StatusPublisher(CallbackAsyncSerial* cmd_serial);
    void Refresh();
    void UpdateStatus(const char *data, unsigned int len);
    void SetSensorType(int sensor_type);
    SENSOR_STATUS sensor_status;

private:
    bw_env_sensors::EnvSensors  mEnvData_;
    ros::NodeHandle mNH;
    ros::Publisher mEnvStatusPub;
    ros::Publisher mEnvFlagPub;
    bool mbUpdated_;
    boost::mutex mMutex;
    bool smoke_ready_;
    bool lel_ready_;
    int sensor_type_;
    CallbackAsyncSerial* cmd_serial_;
};

} //bw_env_sensors

#endif // BWENVSENSORS_H
