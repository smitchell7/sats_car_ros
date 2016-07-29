#!/usr/bin/env python

import rospy 
from numpy import arctan2, sin, cos, radians, arcsin, sqrt, degrees, sign
import numpy as np
from std_msgs.msg import Float32
from sats_car_ros.msg import CarCommand
from sats_car_ros.msg import GpsData 
from sats_car_ros.msg import ControllerCommand
from sats_car_ros.msg import SensorData
from sats_car_ros.msg import MasterCommand
from sats_car_ros.msg import KalmanData
from geometry_msgs.msg import Point32
from pixy_msgs.msg import BlockInfo
from time import time


w = 0.
vel   = 0.0
d_vel = 0.0
dist  = 0.0
theta = 0.0
vel_des = 1.0
vel_int_error = 0.0
vel_error = 0.0
vel_int_accel = 0.0

controller_command = ControllerCommand()

def controller_init():
  rospy.init_node('controller',anonymous=True)
  rospy.loginfo(path)
  setGlobals()
  rospy.Subscriber("tm4c",SensorData,sensor_callback)
  rospy.Subscriber('car_command',CarCommand,car_callback)
  rospy.Subscriber('Block_info',BlockInfo,pixy_callback)
  rospy.Subscriber('/master',MasterCommand,master_callback)
  rospy.Subscriber('/gps',GpsData,their_gps_callback)
  rospy.Subscriber('gps',GpsData,my_gps_callback)
  rospy.Subscriber('set_vel',Float32,set_vel_callback)
## ensure the motors shut off when you stop the script. 
  rospy.on_shutdown(shutdown)

def controller(): 
  global controller_command
  global error_old
  global bearing_publisher 
  bearing_publisher = rospy.Publisher('heading_info', Point32, queue_size = 3)
  controller_command_publisher = rospy.Publisher('controller_command', ControllerCommand, queue_size = 1)
  rospy.loginfo('in controller')
  rate = rospy.Rate(LOOP_RATE) 
  loop_counter = 0
  (pwmL,pwmR)= (0.0,0.0)
  while not rospy.is_shutdown():
    controller_command.header.stamp = rospy.Time.now()

    ## begin high level
    if car_command.HL_active:
      # execute one in 10 iterations through the loop
      if not loop_counter % hl_rate:
        (controller_command.accel,
          controller_command.vel,
          controller_command.curvature,
          controller_command.velL,
          controller_command.velR) = high_level()
    ## end high level 

    ## begin low level 
    if car_command.LL_active:
      (pwmL,left_motor_params['error_integral'],left_motor_params['error_old'],left_motor_params['pid_vel'],left_motor_params['error_old2']) = \
        low_level(controller_command.velL, sensor_data.left.velocity, left_motor_params)
      (pwmR,right_motor_params['error_integral'],right_motor_params['error_old'],right_motor_params['pid_vel'],right_motor_params['error_old2']) = \
        low_level(controller_command.velR, sensor_data.right.velocity, right_motor_params)
    if car_command.PWM_mode:
      (pwmL,pwmR) = (car_command.pwmL,car_command.pwmR)

    try: 
      (controller_command.pwmL, controller_command.pwmR) = (pwmL,pwmR)
    except: 
      (pwmL,pwmR)= (0.0,0.0)
      rospy.loginfo('error writing pwm')
      (controller_command.pwmL, controller_command.pwmR) = (pwmL,pwmR)
    ## end low level 

    ## Master on here ##
    if not master_on: 
      (pwmL,pwmR)= (0.0,0.0)
      left_motor_params['error_integral'] = 0.
      right_motor_params['error_integral'] = 0.
      left_motor_params['error_old'] = 0.
      right_motor_params['error_old'] = 0.
      left_motor_params['pid_vel'] = 0.
      right_motor_params['pid_vel'] = 0.
      left_motor_params['error_old2'] = 0.
      right_motor_params['error_old2'] = 0.
    setpwm(pwmL,pwmR)

    # if the car is running in pwm / input mode, we don't want to pollute the stream
    controller_command_publisher.publish(controller_command)
    loop_counter += 1
    rate.sleep()

################### state estimator for HL controller ####################3
def ekf(*arg):
  # implementation of a continuous-time extended Kalman filter
  # also has a 3-sigma bound on sensor data. 

  ## usage:
  # the first time it is called with two input arguments:
  #   it sets initial data and runs the ekf
  # any time after that it is called with two input arguments:
  #   it just runs the ekf
  # call with one argument to
  #   do nothing and reset internal parameters

  ## input arguments
  # the two input arguments should be (u,y)
  # u is the input vector (2x1 velocity input for us)
  # y is the sensor/output vector (2x1, lidar ; pixy for us)
  # the u argument (control input) is not used at all in our case.

  # initialize some internal data for the Kalman Filter if it's run for the first time
  if (hasattr(ekf, 'P') == False ) or (len(arg) == 1 ):
    Q11 = rospy.get_param('/params/Q11', 0.0)
    Q22 = rospy.get_param('/params/Q22', 200.0)
    Q33 = rospy.get_param('/params/Q33', 0.0)
    Q44 = rospy.get_param('/params/Q44', 5000.0)
    R11 = rospy.get_param('/params/R11', 0.0001)
    R22 = rospy.get_param('/params/R22', 1)
    ekf.Plim_r = rospy.get_param('/params/Plim_r', 100)
    ekf.Plim_th = rospy.get_param('/params/Plim_th', 1000)

    ekf.x = np.matrix('0.5 ; 0; 0; 0') # kalman filter estimated state
    ekf.P = 0.001*np.identity(len(ekf.x)) # estimation error covariance r2_lidar r2dot_lidar th2_pixy
    ekf.Q = np.diag(np.matrix([Q11, Q22, Q33, Q44]).A1) # process noise variance
    ekf.R = np.diag(np.matrix([R11, R22]).A1) # sensor noise variance: r2_lidar th2_pixy
    ekf.Ts = 1/LOOP_RATE*hl_rate/10; # kalman filter time increment
    ekf.pub = rospy.Publisher('kalman_data', KalmanData, queue_size = 1)

  if len(arg) == 1 :
    return (ekf.x,ekf.P)
  else:
    u = arg[0]
    y = arg[1]
  

  A = np.matrix('0 1 0 0 ; 0 0 0 0; 0 0 0 1; 0 0 0 0')

  # if states are going unbounded, reset  ekf
  if np.linalg.norm(ekf.P[[0,1],[0,1]],2) > ekf.Plim_r:
    ekf.x[0,0] = u
    ekf.x[1,0] = 0
  if np.linalg.norm(ekf.P[[2,3],[2,3]],2) > ekf.Plim_th:
    ekf.x[2,0] = 0
    ekf.x[3,0] = 0

  # do the predict step and update covariance with a granularity of 10 
  for i in np.arange(0,1.0/LOOP_RATE*hl_rate,ekf.Ts):
    ekf.x = ekf.x + ekf.Ts*A*ekf.x
    # A = J_f(ekf.x,u)
    ekf.P = ekf.P + ekf.Ts*(A*ekf.P + ekf.P*np.transpose(A) + ekf.Q)
  
  # find expected output covariance S
  C = np.matrix('1 0 0 0; 0 0 1 0') # J_h(ekf.x,u)
  S =  ekf.R + C*ekf.P*np.transpose(C) # output covariance
  
  # filter out the false positives.
  for i in range(2,len(y)):
    if abs(y[1,0] - ekf.x[2,0]) > abs(y[i,0] - ekf.x[2,0]):
      y[1,0] = y[i,0]

  # check if innovation is within 3-sigma bounds
  inov = y[[0,1],0] - C*ekf.x
  y_bound = 3*np.transpose([np.sqrt(np.diagonal(S))])
  valid = np.greater(y_bound,np.absolute(inov))
  if (abs(y[1,0]) > 120):
    valid[1,0] = False

  # ignore that sensor value by adjusting rows of C (ignore bad sensor data)
  C = np.diag(np.array(np.transpose(valid))[0])*C

  # find new lower dimensional Kalman gain L and update P based on new L and C
  L = ekf.P*np.transpose(C) * np.linalg.inv(S)
  ekf.x = ekf.x + L* np.multiply(inov,valid)
  ekf.P = (np.identity(len(ekf.x)) - L*C)*ekf.P
    
  # P should diverge for that state until a new sensor reading within the 3-sigma
  # bound is recorded. But the 3-sigma bound also starts to diverge. Kind of
  # contradictory, but as long as we don't get bad data for too long it should do
  # fine (at least in steady state.)

  # print(y_bound)
  # print(inov)
  # print(valid)
  # print(C)
  # print(" ")
  # print(ekf.x)
  msg = KalmanData()
  msg.r = ekf.x[0,0]
  # msg.rdot = ekf.x[1,0]
  msg.rdot = len(y)
  msg.theta = ekf.x[2,0]
  # msg.theta = y[1,0]
  msg.thetadot = ekf.x[3,0]
  msg.covariance = np.float32(ekf.P.flatten().tolist())[0]
  ekf.pub.publish(msg)
  return ekf.x

################### controllers ####################3
#### High level #####
def high_level(): 
  global vel_int_accel
  
  vel_actual = (sensor_data.left.velocity + sensor_data.right.velocity) / 2.0 
  if GPS_MODE:
    (dist,theta) = set_gps_info()
    d_vel = 0.
  else:
    # (dist,theta,d_vel) = (sensor_data.front.distance, pixy_theta, sensor_data.front.velocity)
    y = np.concatenate((np.matrix(sensor_data.front.distance), pixy_theta), axis=0 )
    xhat = ekf(hl_gains['nom_dist'] + hl_gains['h']*vel_actual, y )
    (dist,theta,d_vel) = (xhat[0,0], radians(xhat[2,0]), xhat[1,0])
  
  # at this point, theta should be in radians

  # select controller type. We could use an integer instead. 
  if car_command.VEL_mode: 
    t = (controller_command.header.stamp.secs + 
      0.000000001 * controller_command.header.stamp.nsecs)
    try:
      cos_period = 1. / car_command.d_period
    except:
      cos_period = 0.
    vel = car_command.command_velocity + car_command.d_amplitude * cos(t*2.*3.14*cos_period )
    accel = 0.
  else:
    accel = longitudinal( dist, vel_actual, d_vel)
    # vel_int_accel += accel * hl_rate / LOOP_RATE
    vel_int_accel += accel * (hl_rate/ LOOP_RATE)
    if abs(vel_int_accel) > 1.5 :
      vel_int_accel = sign(vel_int_accel)*1.5
    vel = vel_int_accel

  if car_command.curv_auto:
    curv = car_command.curv
  else: 
    curv = pursuit( dist, theta)

  (velL, velR) = steering(vel,curv)
  return (accel, vel, curv, velL, velR)

def longitudinal(dist,vel,d_vel):
  accel = hl_gains['kp']*(dist - hl_gains['nom_dist'] - hl_gains['h']*vel) + hl_gains['kd']*d_vel
  return accel

def pursuit(dist,theta):
  kurv = 2.*sin(theta)/dist
  return kurv

def steering(vel,curvature): 
  velL = vel * (1 + curvature * hl_gains['L']/2.0)
  velR = vel * (1 - curvature * hl_gains['L']/2.0)
  return (velL, velR)

#### Low level #####
def low_level(vel_des, vel_cur,motor_params):
  error = vel_des - vel_cur

  error_old = motor_params['error_old']
  error_old2 = motor_params['error_old2']
  pid_vel = motor_params['pid_vel']

  if abs(vel_des) > 0.05:
    u_diff = ll_gains['kp'] *(error-error_old)  +\
               ll_gains['ki']/LOOP_RATE * (error) + \
               ll_gains['kd']*LOOP_RATE*(error - 2*error_old + error_old2)
    pid_vel += u_diff
    
    # implement antiwindup
    if abs(motor_params['k']*pid_vel) > ll_gains['pwm_sat'] :
      pid_vel = sign(pid_vel) * ll_gains['pwm_sat'] / motor_params['k']
    pwm = motor_params['k']*pid_vel
#    rospy.loginfo(u_diff)
  else:
    pwm = 0.0
#  info = {'pid_vel',pid_vel,'error_old',error_old,'pwm',pwm,'vel_des',vel_des}
#  rospy.loginfo([pid_vel,error_old,pwm,vel_des])
  error_old2 = error_old
  error_old = error
  # return (pwm, error_integral,0,0)
  return (pwm, 0, error_old, pid_vel, error_old2)

def setpwm(cmdL,cmdR):
    global pwm0
    global pwm1
    # saturate pwm commands. 
    if abs(cmdL) > 0.05:
      cmdL += sign(cmdL) * left_motor_params['deadband']
    if abs(cmdR) > 0.05:
      cmdR += sign(cmdR) * right_motor_params['deadband']
    if abs(cmdL) > ll_gains['pwm_sat']:
      cmdL = ll_gains['pwm_sat']*sign(cmdL) 
    if abs(cmdR) >ll_gains['pwm_sat']:
      cmdR =ll_gains['pwm_sat']*sign(cmdR) 
    pwm0.write(str(int(1500000+cmdL*500000))) # set pwm
    pwm1.write(str(int(1500000-cmdR*500000))) # set pwm

##################### Callbacks ##############################

GPS_FRONT = GpsData()
def their_gps_callback(data):
  global GPS_FRONT
  GPS_FRONT = data
GPS_SELF  = GpsData()
def my_gps_callback(data):
  global GPS_SELF
  GPS_SELF  = data

sensor_data = SensorData()
def sensor_callback(data): 
  global sensor_data
  sensor_data = data  

pixy_theta = 0.
def pixy_callback(data): 
  global pixy_theta
  #pixy_theta = arctan2(data.x - 160, data.y) * pixy_offset['pt']
  #rospy.loginfo(pixy_theta)
    # pixy_theta = arctan2(data.x - 160, 200 - data.y)
  pixy_theta = np.transpose(np.matrix(data.all_thetas))
  pixy_theta = pixy_correction_ofs + pixy_correction_gain* pixy_theta
  # pixy_theta = data.theta
  #global pixy_dist
##  pixy_dist = a*exp(-b*x)+c 
#  pixy_dist = pixy_offset('pa') * np.exp(-pixy_offset('pb')*data.height) + pixy_offset('pc')

car_command = CarCommand()
car_command.HL_active = False 
car_command.HL_atk_mode = False
car_command.VEL_mode = False
car_command.PWM_mode = False
def car_callback(data):
  global car_command
  if car_command.HL_atk_mode != data.HL_atk_mode: 
    setGlobals() 
  car_command = data
  rospy.loginfo('car command received')

def set_vel_callback(data): 
  global vel_des
  vel_des = data.data

master_on = False 
def master_callback(data): 
  global master_on
  master_on = data.master_on
  rospy.loginfo(master_on)
  if data.update_param: 
    setGlobals()

def setGlobals(): 
  global hl_gains
  global GPS_MODE 
  global LOOP_RATE 
  global hl_rate

  nom_dist = rospy.get_param('/params/nom_dist',1.) 
  L    = rospy.get_param('/params/L', 0.31)
  if car_command.HL_atk_mode:
    kp = rospy.get_param('/params/kpATK',1.)
    kd = rospy.get_param('/params/kdATK',1.)
    h  = rospy.get_param('/params/hATK',1.)
  else:
    kp = rospy.get_param('/params/kp',1.)
    kd = rospy.get_param('/params/kd',1.)
    h  = rospy.get_param('/params/h',1.)

  GPS_MODE = rospy.get_param('motor_gains/gps_mode',False) 
  LOOP_RATE= rospy.get_param('/params/controller_rate',50.)
  hl_rate = rospy.get_param('/params/high_level_rate',10.)
  lookahead = rospy.get_param('/params/lookahead',-6)
  hl_gains = {'kp': kp, 'kd': kd, 'h': h, 'nom_dist': nom_dist, 'L': L, 'lookahead': lookahead}

  global ll_gains
  global left_motor_params
  global right_motor_params
  global error_int
  global vel_int_accel

  error_int = 0.

  # get_param checks if a value is on the server. if it's not, it uses default. 
  aL    = rospy.get_param('motor_gains/aL', -2.7)
  aR    = rospy.get_param('motor_gains/aR', -2.7)
  bL     = rospy.get_param('motor_gains/bL', 0.8)
  bR     = rospy.get_param('motor_gains/bR', 0.8)
  deadbandL   = rospy.get_param('motor_gains/deadbandL', 0.13)
  deadbandR   = rospy.get_param('motor_gains/deadbandR', 0.13)
  ll_kp     = rospy.get_param('/params/ll_kp', 0.03)
  ll_ki     = rospy.get_param('/params/ll_ki', 0.01)
  ll_kd     = rospy.get_param('/params/ll_kd', 0.0025)
  pwm_sat   = rospy.get_param('/params/pwm_sat', 0.6)

  #global pixy_offset
  #pt = rospy.get_param('pixy_const',0.55)
  #pa = rospy.get_param('pixy_dist_a',75.52)
  #pb = rospy.get_param('pixy_dist_c',1.424)
  #pc = rospy.get_param('pixy_dist_c',6.154)
  
  #pixy_offset = {'pt': pt, 'pa': pa,'pb': pb,  'pc': pc} 

  global pixy_correction_ofs
  global pixy_correction_gain
  pixy_correction_ofs =  rospy.get_param('motor_gains/pixy_ofs',0.0)
  pixy_correction_gain = rospy.get_param('motor_gains/pixy_gain', 0.55)
  rospy.loginfo(pixy_correction_gain)

  ll_gains = {'kp': ll_kp, 'ki': ll_ki, 'kd': ll_kd, 'pwm_sat': pwm_sat}
  kL = -aL / (36. * bL)
  kR = -aR / (36. * bR)
  left_motor_params = {'a': aL, 'b': bL, 'k': kL, 'deadband': deadbandL, 'error_integral': 0., 'error_old': 0., 'pid_vel': 0., 'error_old2': 0.}
  right_motor_params= {'a': aR, 'b': bR, 'k': kR, 'deadband': deadbandR, 'error_integral': 0., 'error_old': 0., 'pid_vel': 0., 'error_old2': 0.}

  ekf(0) # reset Kalman filter
  vel_int_accel = 0.0

#  rospy.loginfo([ll_gains,hl_gains,left_motor_params,right_motor_params])


################## library style functions #################
# this calculates gps range and bearing
def haversine(lat1, lon1, lat2, lon2):
  R = 6372800 # Earth radius in meters
 
  dLat = radians(lat2 - lat1)
  dLon = radians(lon2 - lon1)
  lat1 = radians(lat1)
  lat2 = radians(lat2)
 
  a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
  c = 2*arcsin(sqrt(a))
  distance = R * c

  y = sin(dLon) * cos(lat2)
  x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
  bearing = arctan2(y,x)
  return (distance, bearing)


def wrapTo180(x):
  x = (x+180) % 360
  if x < 0:
    x += 360
  return x - 180
bearing_msg = Point32()
def set_gps_info():
  # GPS leader mode
  self_pos = np.array([[GPS_SELF.gps.x], [GPS_SELF.gps.y]])
  (pg,pc) = goalSelect(self_pos, hl_gains['lookahead'])
  
  # GPS follower mode
  self_heading = radians(GPS_SELF.ypr.x) 
  (dist,bearing) = haversine(self_pos[0], self_pos[1], pg[0], pg[1])
  theta = bearing-self_heading 

  bearing_publisher.publish(dist, wrapTo180(bearing*180.0/3.14), wrapTo180(theta*180.0/3.14))
  #rospy.loginfo([degrees(theta),degrees(bearing),degrees(self_heading),dist])
  return (dist,theta) 


# define function constants
path = np.genfromtxt('/home/sats/code/path_norm/path.csv', delimiter=',')
def goalSelect(pr,l):
    n = path.shape[1] #number of measurements
    # INPUTS:
    # path = path to follow (2-by-measurements)
    # pr = current position of robot (2-by-1)
    # l = look-ahead distance (number of indices in path data)
    #
    # OUTPUTS:
    # pg = point on path robot should pursue
    # pc = point on path closest to robot's current position

    
    # calculate manhattan distance from p0 to every point on path
#    mpd = [111069.,83162.]
    d = np.absolute(pr[0] - path[0,:])*1.33 + np.absolute(pr[1] - path[1,:])
    Ipc = d.argmin() #find index to point on path with smallest distance

    pc = path[:,Ipc].reshape(2,1) #closest point

    # get goal point
    Ipg = np.mod(Ipc + l, n)
    pg = path[:,Ipg].reshape(2,1) #goal point

    return pg, pc


def shutdown():
  pwm0.write(str(1500000))
  pwm1.write(str(1500000))
  pwm0.close()
  pwm1.close()

pwm0 = open('/sys/class/pwm/pwmchip0/pwm0/duty_cycle','wb',0)
pwm1 = open('/sys/class/pwm/pwmchip0/pwm1/duty_cycle','wb',0)
pwm0 = open('/dev/null','wb',0)
pwm1 = open('/dev/null','wb',0)


############ Execution of main code ####################
if __name__=='__main__':
  try:
      controller_init()
      controller()
  except rospy.ROSInterruptException:
    pass


