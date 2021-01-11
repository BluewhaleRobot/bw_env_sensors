#!/usr/bin/env python
#encoding=utf-8

import rospy
from std_msgs.msg import Bool, String
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
import time
import rosservice
import subprocess
import os
import threading
from bw_env_sensors.msg import EnvSensors
import requests
import json

NAV_STATUS = 0
LOOP_STATUS = 0

STATUS_LOCK = threading.Lock()

ENV_DATA_LOCK = threading.Lock()

AUDIO_PUB = None

GALILEO_PUB = None

TASK_LIST = list()

SMOKE_WARNING = 300
LEL_WARNING = 20

ENV_SENSOR_DATA = EnvSensors()

def status_update_cb(status):
    global NAV_STATUS,STATUS_LOCK,LOOP_STATUS
    with STATUS_LOCK:
        NAV_STATUS = status.navStatus
        LOOP_STATUS = status.loopStatus

def update_envSensor(sensorData):
    global ENV_SENSOR_DATA,ENV_DATA_LOCK
    with ENV_DATA_LOCK:
        ENV_SENSOR_DATA.header = sensorData.header
        ENV_SENSOR_DATA.temperature = sensorData.temperature #Centigrade
        ENV_SENSOR_DATA.rh = sensorData.rh #relative humidity %RH
        ENV_SENSOR_DATA.smoke = sensorData.smoke #ppm
        ENV_SENSOR_DATA.pm1_0 = sensorData.pm1_0 #ug/m^3
        ENV_SENSOR_DATA.pm2_5 = sensorData.pm2_5 #ug/m^3
        ENV_SENSOR_DATA.pm10 = sensorData.pm10 #ug/m^3
        ENV_SENSOR_DATA.lel = sensorData.lel #ppm
        ENV_SENSOR_DATA.noise = sensorData.noise #db

def dealEnvData():
    global NAV_STATUS,STATUS_LOCK,LOOP_STATUS,ENV_SENSOR_DATA,ENV_DATA_LOCK,GALILEO_PUB
    ENV_DATA_LOCK.acquire()
    need_play = False
    if ENV_SENSOR_DATA.smoke >= SMOKE_WARNING or ENV_SENSOR_DATA.lel >= LEL_WARNING:
        need_play = True

    cmd1 = "aplay /home/xiaoqiang/Documents/ros/src/bw_env_sensors/src/bw_env_sensors/error.wav"
    if need_play:
        subprocess.Popen(cmd1,shell=True)
        STATUS_LOCK.acquire()
        nav_status = NAV_STATUS
        loop_status = LOOP_STATUS
        STATUS_LOCK.release()
        #取消循环
        galileo_cmds = GalileoNativeCmds()
        galileo_cmds.data = 'm' + chr(0x06)
        galileo_cmds.length = len(galileo_cmds.data)
        galileo_cmds.header.stamp = rospy.Time.now()
        if loop_status == 1 and nav_status == 1:
            GALILEO_PUB.publish(galileo_cmds)
        #取消当前任务
        galileo_cmds = GalileoNativeCmds()
        galileo_cmds.data = 'i' + chr(0x02)
        galileo_cmds.length = len(galileo_cmds.data)
        galileo_cmds.header.stamp = rospy.Time.now()
        GALILEO_PUB.publish(galileo_cmds)
        #停止导航
        galileo_cmds = GalileoNativeCmds()
        galileo_cmds.data = 'm' + chr(0x04)
        galileo_cmds.length = len(galileo_cmds.data)
        galileo_cmds.header.stamp = rospy.Time.now()
        GALILEO_PUB.publish(galileo_cmds)
        if nav_status == 1:
            try:
                requests.get("http://127.0.0.1:3546/api/v1/navigation/stop")
            except Exception as e:
                rospy.logwarn(e)
    ENV_DATA_LOCK.release()

if __name__ == "__main__":

    rospy.init_node("env_sensor_monitor_server")

    AUDIO_PUB = rospy.Publisher("/xiaoqiang_tts/text", String, queue_size=10)

    GALILEO_PUB = rospy.Publisher("/galileo/cmds", GalileoNativeCmds, queue_size=5)

    status_sub = rospy.Subscriber("/galileo/status", GalileoStatus, status_update_cb)

    envSensor_sub = rospy.Subscriber("/bw_env_sensors/EnvSensorData", EnvSensors, update_envSensor)

    time.sleep(1)

    SMOKE_WARNING = rospy.get_param("~SMOKE_WARNING", 300)
    LEL_WARNING = rospy.get_param("~LEL_WARNING", 20)

    while not rospy.is_shutdown():
        #任务最多每分钟执行一次
        dealEnvData()
        time.sleep(2)
