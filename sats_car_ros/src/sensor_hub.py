#!/usr/bin/env python

from serial import Serial
import struct
import rospy
from std_msgs.msg import Float32
from sats_car_ros.msg import SensorData
from sats_car_ros.msg import GpsData

ser = Serial('/dev/ttyS5', 460800)

sensor_msg = SensorData()
def sensorHub():
    rospy.init_node('sensorHub',anonymous=True)
    setGlobals()
    global pub
    global gps_pub
    pub = rospy.Publisher('tm4c',SensorData, queue_size=1)
    if GPS_MODE:
      gps_pub = rospy.Publisher('gps',GpsData,queue_size=1)
    rate = rospy.Rate(LOOP_RATE)

    while not rospy.is_shutdown():
        # when the data is available from UART, send it. 
        for i in range(5*packet_size):
#          if ser.inWaiting() > packet_size/8:
          read_packet()
        ser.flushInput()
#          rospy.sleep(0.01)
        rate.sleep()

def read_packet():
  if ord(ser.read()) == 0x7E:
    #checksum = 0x7E
#    data = ""
    data = ser.read(packet_size+2)
#    for i in range(packet_size):
#      byte = ser.read()
#      checksum += ord(byte)
#      data += byte
    #checksum += ord(ser.read())
    #checksum += sum(bytearray(data))
    checksum = crc(data[:-2])
    cs2 = struct.unpack('H',data[-2:])[0]
    if checksum == cs2:
      good_packet= struct.unpack(struct_format, data[:-2])
      copy_msg(good_packet)
      pub.publish(sensor_msg)
      global counter
      if GPS_MODE and not (counter % 5): 
        gps_pub.publish(gps_msg)
      counter +=1
      return 1
    else:
      rospy.loginfo('error on read %x',checksum)
      ser.flushInput()
      return -1
counter = 0

def copy_msg(data): 
    global sensor_msg
    
    sensor_msg.header.stamp = rospy.Time.now()
    sensor_msg.front.distance = data[0]
    sensor_msg.front.velocity = data[1]
    sensor_msg.rear.distance  = data[2]
    sensor_msg.rear.velocity  = data[3]
    sensor_msg.right.distance =-data[4] * performance_offset
    sensor_msg.right.velocity =-data[5] * performance_offset
    sensor_msg.left.distance  =-data[6] * performance_offset
    sensor_msg.left.velocity  =-data[7] * performance_offset
    
    if GPS_MODE: 
      global gps_msg
      gps_msg.header.stamp = sensor_msg.header.stamp
      gps_msg.ypr.x = data[8]
      gps_msg.ypr.y = data[9] 
      gps_msg.ypr.z = data[10] 
      gps_msg.angular.x = data[11]
      gps_msg.angular.y = data[12] 
      gps_msg.angular.z = data[13] 
      gps_msg.gps.x = data[14]
      gps_msg.gps.y = data[15] 
      gps_msg.gps.z = data[16] 
      gps_msg.ned.x = data[17]
      gps_msg.ned.y = data[18] 
      gps_msg.ned.z = data[19] 
      gps_msg.accel.x = data[20]
      gps_msg.accel.y = data[21] 
      gps_msg.accel.z = data[22] 
def setGlobals(): 
  global GPS_MODE 
  global LOOP_RATE
  global performance_offset
  performance_offset = rospy.get_param('motor_gains/performance_car',1)
  GPS_MODE = rospy.get_param('motor_gains/gps_mode',False)
  LOOP_RATE = rospy.get_param('/params/sensor_rate',50)

  global struct_format
  global packet_size

  struct_format = 'ffffffff' + 'ffffff' + 'ddd' + 'ffffff'
  packet_size = 4 * (len(struct_format) + 3) 

  if not GPS_MODE:
    struct_format = struct_format[:8]
    packet_size = 4*len(struct_format)
  if GPS_MODE: 
    global gps_msg
    gps_msg = GpsData()
  rospy.loginfo([GPS_MODE,LOOP_RATE,performance_offset])
def crc(data):
    crc = 0
    for byte in data: 
        crc  = (crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
        crc  = crc & 0xffff
    return crc
        

def crc(data):
    crc = 0
    for byte in data: 
        crc  = (crc >> 8) | (crc << 8);
        crc ^= struct.unpack('B',byte)[0]
        crc ^= (crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
        crc  = crc & 0xffff
    return int(crc)
        

if __name__=='__main__':
    sensorHub()

