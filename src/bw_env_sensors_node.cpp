#include "bw_env_sensors/AsyncSerial.h"
#include <iostream>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include "bw_env_sensors/bw_env_sensors.h"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"welcome to bw_env_sensors serial server,please feel free at home!"<<endl;

    ros::init(argc, argv, "bw_env_sensors_serial_server");
    ros::start();

    //获取串口参数
    std::string port;
    ros::param::param<std::string>("~port", port, "/dev/bwEnvSensor");
    int baud;
    ros::param::param<int>("~baud", baud, 9600);
    cout<<"port:"<<port<<" baud:"<<baud<<endl;

    try {
        CallbackAsyncSerial serial(port,baud);
        bw_env_sensors::StatusPublisher bwEnvSensors_server(&serial);
        serial.setCallback(boost::bind(&bw_env_sensors::StatusPublisher::UpdateStatus,&bwEnvSensors_server,_1,_2));

        const char data_query1[8] = {(char)0x01,(char)0x03,(char)0x00,(char)0x01,(char)0x00,(char)0x0b,(char)0x55,(char)0xcd};//获取前11个传感器数据
        const char data_query2[8] = {(char)0x02,(char)0x03,(char)0x00,(char)0x06,(char)0x00,(char)0x01,(char)0x64,(char)0x38};//获取lel传感器数据
        ros::Rate r(1);//发布周期为1hz
        while (ros::ok())
        {
            bwEnvSensors_server.Refresh();//定时发布状态
            if(serial.errorStatus() || serial.isOpen()==false)
            {
                cerr<<"Error: serial port closed unexpectedly"<<endl;
                break;
            }
            bwEnvSensors_server.SetSensorType(1);
            serial.write(data_query1,8);
            usleep(200000);//延时200MS，等待数据上传
            bwEnvSensors_server.SetSensorType(2);
            serial.write(data_query2,8);
            r.sleep();
        }
        quit:
        serial.close();

    } catch (std::exception& e) {
        cerr<<"Exception: "<<e.what()<<endl;
    }
    ros::shutdown();
    return 0;
}
