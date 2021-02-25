#include "bw_env_sensors/AsyncSerial.h"
#include "bw_env_sensors/bw_env_sensors.h"
#include "bw_env_sensors/crc16.h"
#include <algorithm>
namespace bw_env_sensors
{
StatusPublisher::StatusPublisher(CallbackAsyncSerial* cmd_serial)
                                :cmd_serial_(cmd_serial)
{
  mbUpdated_=false;

  sensor_status.temperature_org = 2000;
  sensor_status.rh_org = 0;
  sensor_status.smoke_org = 0;
  sensor_status.pm1_0_org = 0;
  sensor_status.pm2_5_org = 0;
  sensor_status.pm10_org = 0;
  sensor_status.lel_org = 0;
  sensor_status.noise_org = 0;

  mEnvData_.header.stamp = ros::Time::now();
  mEnvData_.header.frame_id = "EnvSensor";
  mEnvData_.temperature = 0;
  mEnvData_.rh = 0;
  mEnvData_.smoke = 0;
  mEnvData_.pm1_0 = 0;
  mEnvData_.pm2_5 = 0;
  mEnvData_.pm10  = 0;
  mEnvData_.lel = 0;
  mEnvData_.noise =  0;

  sensor_type_ = 0;
  smoke_ready_ = false;
  lel_ready_ = false;

  mEnvStatusPub = mNH.advertise<bw_env_sensors::EnvSensors>("/bw_env_sensors/EnvSensorData",1,true);
  mEnvFlagPub = mNH.advertise<std_msgs::Int32>("/bw_env_sensors/StatusFlag",1,true);
}

void StatusPublisher::Refresh()
{
  static int update_num =0;
  if(update_num >6000 ) update_num = 6000;
  update_num ++;

  std_msgs::Int32 flag;
  flag.data = 0;
  if(mbUpdated_ )
  {
    boost::mutex::scoped_lock lock(mMutex);
    if(smoke_ready_)
    {
      mEnvData_.temperature = sensor_status.temperature;
      mEnvData_.rh = sensor_status.rh;
      if(update_num >60 || sensor_status.smoke<500)
      {
        mEnvData_.smoke = sensor_status.smoke;
      }
      else
      {
        mEnvData_.smoke = 0;
      }
      mEnvData_.pm1_0 = sensor_status.pm1_0;
      mEnvData_.pm2_5 = sensor_status.pm2_5;
      mEnvData_.pm10  = sensor_status.pm10;
      mEnvData_.noise = sensor_status.noise;

      flag.data += 1;
      smoke_ready_ = false;
    }

    if(lel_ready_)
    {
      mEnvData_.lel = sensor_status.lel;
      flag.data += 2;
      lel_ready_ = false;
    }

    mEnvData_.header.stamp = ros::Time::now();
    mEnvData_.header.frame_id = "EnvSensor";

    mEnvStatusPub.publish(mEnvData_);
    mbUpdated_ = false;
  }
  //0 表示没有数据，1表示烟雾传感器正常，2表示可燃气体传感器正常，3表示两个传感器都正常
  mEnvFlagPub.publish(flag);
}

void StatusPublisher::UpdateStatus(const char *data, unsigned int len)
{
  int i=0,j=0;
  int * receive_byte;
  static std::deque<unsigned char>  cmd_string_buf={0x01,0x00,0x16,0x00,0x01,0x00,0x02,0x00,0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x08,0x00,0x09,0x00,0x0a,0x00,0x0b,0x00,0x00};
  static std::deque<unsigned char>  cmd_string_buf2={0x02,0x00,0x02,0x00,0x00,0xfc,0x44};
  unsigned char current_str=0x00;

  for(i=0;i<len;i++)
  {
    current_str=data[i];

    if(sensor_type_ == 1)
    {
      cmd_string_buf.pop_front();
      cmd_string_buf.push_back(current_str);
      //std::cout<< std::hex  << (int)current_str <<std::endl;
      // ROS_ERROR("current %d %d",data[i],i);
      if((cmd_string_buf[0]!=0x01 || cmd_string_buf[1]!=0x03 || cmd_string_buf[2]!=0x16)) //校验包头
      {
        continue;
      }
      //校验crc16
      uint8_t crc_hl[2];
      unsigned char cmd_string_buf_copy[25] = {cmd_string_buf[0],cmd_string_buf[1],cmd_string_buf[2],cmd_string_buf[3],cmd_string_buf[4],cmd_string_buf[5],cmd_string_buf[6],
                                               cmd_string_buf[7],cmd_string_buf[8],cmd_string_buf[9],cmd_string_buf[10],cmd_string_buf[11],cmd_string_buf[12],cmd_string_buf[13],
                                               cmd_string_buf[14],cmd_string_buf[15],cmd_string_buf[16],cmd_string_buf[17],cmd_string_buf[18],cmd_string_buf[19],cmd_string_buf[20],
                                               cmd_string_buf[21],cmd_string_buf[22],cmd_string_buf[23],cmd_string_buf[24]};
      bw_env_sensors::CRC16CheckSum(cmd_string_buf_copy, 25, crc_hl);
      // ROS_ERROR("crc %x %x , in %x %x",crc_hl[0],crc_hl[1],cmd_string_buf[25],cmd_string_buf[26]);
      if(cmd_string_buf[25]!=crc_hl[0] || cmd_string_buf[26]!=crc_hl[1] )
      {
        continue;
      }
      // ROS_ERROR("receive one package!");
      //有效包，开始提取数据
      {
        boost::mutex::scoped_lock lock(mMutex);

        for(int i=1;i<=11;i++)
        {
          if(cmd_string_buf[(i-1)*2+3+1] == i) continue;
          switch (i)
          {
            case 1:
            {
              sensor_status.temperature_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 2:
            {
              sensor_status.rh_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 3:
            // {
            //   sensor_status.illuminance_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            // }
            break;
            case 6:
            {
              sensor_status.noise_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 7:
            {
              sensor_status.pm1_0_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 8:
            {
              sensor_status.pm2_5_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 9:
            {
              sensor_status.pm10_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
            case 11:
            {
              sensor_status.smoke_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
            break;
          }
          sensor_status.temperature = (sensor_status.temperature_org-2000)/100.0f;   //Centigrade
          sensor_status.rh = sensor_status.rh_org/100.0f;            //relative humidity %RH
          sensor_status.smoke = sensor_status.smoke_org;         //ppm

          sensor_status.pm1_0 = sensor_status.pm1_0_org;         //ug/m^3
          sensor_status.pm2_5 = sensor_status.pm2_5_org;         //ug/m^3
          sensor_status.pm10 = sensor_status.pm10_org;          //ug/m^3

          float pm_smoke = (sensor_status.pm1_0 + sensor_status.pm2_5 + sensor_status.pm10)/3.0;
          if(pm_smoke>5)
          {
            sensor_status.smoke = std::max(sensor_status.smoke,pm_smoke);  
          }
          sensor_status.noise = sensor_status.noise_org/10;         //db
        }
        smoke_ready_ = true;
        mbUpdated_ = true;
      }
    }
    else
    {
      if(sensor_type_ == 2)
      {
        cmd_string_buf2.pop_front();
        cmd_string_buf2.push_back(current_str);
        //std::cout<< std::hex  << (int)current_str <<std::endl;
        // ROS_ERROR("current %d %d",data[i],i);
        if((cmd_string_buf2[0]!=0x02 || cmd_string_buf2[1]!=0x03 || cmd_string_buf2[2]!=0x02)) //校验包头
        {
          continue;
        }
        //校验crc16
        uint8_t crc_hl[2];
        unsigned char cmd_string_buf2_copy[5] = {cmd_string_buf2[0],cmd_string_buf2[1],cmd_string_buf2[2],cmd_string_buf2[3],cmd_string_buf2[4]};
        bw_env_sensors::CRC16CheckSum(cmd_string_buf2_copy, 5, crc_hl);
        // ROS_ERROR("crc %x %x , in %x %x",crc_hl[0],crc_hl[1],cmd_string_buf2[5],cmd_string_buf2[6]);
        if(cmd_string_buf2[5]!=crc_hl[0] || cmd_string_buf2[6]!=crc_hl[1] )
        {
          continue;
        }
        // ROS_ERROR("receive one package!");
        //有效包，开始提取数据
        {
          boost::mutex::scoped_lock lock(mMutex);

          sensor_status.lel_org = cmd_string_buf2[3]<<8|cmd_string_buf2[4];

          sensor_status.lel = sensor_status.lel_org/10;   //ppm

          mbUpdated_ = true;
          lel_ready_ = true;
        }
      }
    }

  }
  return;
}

void StatusPublisher::SetSensorType(int sensor_type)
{
  sensor_type_ = sensor_type;
}

}
