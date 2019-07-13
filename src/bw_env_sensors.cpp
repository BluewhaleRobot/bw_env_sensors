#include "bw_env_sensors/AsyncSerial.h"
#include "bw_env_sensors/bw_env_sensors.h"
#include "bw_env_sensors/crc16.h"
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
  sensor_status.illuminance_org = 0;
  sensor_status.noise_org = 0;

  mEnvData_.header.stamp = ros::Time::now();
  mEnvData_.header.frame_id = "EnvSensor";
  mEnvData_.temperature = 0;
  mEnvData_.rh = 0;
  mEnvData_.smoke = 0;
  mEnvData_.pm1_0 = 0;
  mEnvData_.pm2_5 = 0;
  mEnvData_.pm10  = 0;
  mEnvData_.illuminance = 0;
  mEnvData_.noise =  0;

  mEnvStatusPub = mNH.advertise<bw_env_sensors::EnvSensors>("/bw_env_sensors/EnvSensorData",1,true);

}

void StatusPublisher::Refresh()
{
  if(mbUpdated_ )
  {
    boost::mutex::scoped_lock lock(mMutex);
    mEnvData_.header.stamp = ros::Time::now();
    mEnvData_.header.frame_id = "EnvSensor";
    mEnvData_.temperature = sensor_status.temperature;
    mEnvData_.rh = sensor_status.rh;
    mEnvData_.smoke = sensor_status.smoke;
    mEnvData_.pm1_0 = sensor_status.pm1_0;
    mEnvData_.pm2_5 = sensor_status.pm2_5;
    mEnvData_.pm10  = sensor_status.pm10;
    mEnvData_.illuminance = sensor_status.illuminance;
    mEnvData_.noise = sensor_status.noise;
    mEnvStatusPub.publish(mEnvData_);
    mbUpdated_ = false;
  }
}

void StatusPublisher::UpdateStatus(const char *data, unsigned int len)
{
  int i=0,j=0;
  int * receive_byte;
  static std::deque<unsigned char>  cmd_string_buf={0x01,0x03,0x16,0x00,0x01,0x00,0x02,0x00,0x03,0x00,0x04,0x00,0x05,0x00,0x06,0x00,0x07,0x00,0x08,0x00,0x09,0x00,0x0a,0x00,0x0b,0x00,0x00};
  unsigned char current_str=0x00;

  for(i=0;i<len;i++)
  {
      current_str=data[i];
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
          switch (i) {
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
            {
              sensor_status.illuminance_org = cmd_string_buf[(i-1)*2+3]<<8|cmd_string_buf[(i-1)*2+3+1];
            }
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
        sensor_status.illuminance = sensor_status.illuminance_org;   //lx
        sensor_status.noise_org = sensor_status.noise/10;         //db
      }
      mbUpdated_ = true;
    }
  }
  return;
}

}
