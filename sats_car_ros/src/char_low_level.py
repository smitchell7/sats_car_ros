#!/usr/bin/env python

import rospy 
import numpy as np
from numpy import sign
from sats_car_ros.msg import CarCommand
from sats_car_ros.msg import HighLevel
from sats_car_ros.msg import SensorData
from sats_car_ros.msg import UserCommand 
sensorData = SensorData()
velR_ave = 0.0
velL_ave = 0.0
vel_ctr = 0

def find_dbK(F,v,N_TP):

    V = 36 # max voltage

    d_opt = -1
    sval_opt = 1000
    
    M_d = np.transpose(np.concatenate( (np.matrix(v), V*(np.matrix(F)) )  , axis=0))

    for d_test in np.arange(0.05,0.15,0.0025):
        D = np.transpose(np.matrix( [d_test]* N_TP))
        D = np.concatenate( (0*D, D ),axis=1 )

        _, s, _ = np.linalg.svd(M_d-D*V, full_matrices=True)
        if (s.min() < sval_opt):
            d_opt = d_test
            sval_opt = s.min()
    
    D = np.transpose(np.matrix( [d_opt]* N_TP))
    D = np.concatenate( (0*D, D ),axis=1 )
    _, s, Vec = np.linalg.svd(M_d-D*V, full_matrices=True)
    return(d_opt,np.abs(Vec[1,1]/Vec[0,1]))

def find_tau(T,v,db,K,F):

    V = 36 # max voltage
    tau_opt = -1
    J_opt = 1000
    for tau_test in np.arange(0.2,0.5,0.01):
        v_hat = K*V*(F-db)*(1-np.exp(np.multiply(T,-1/tau_test) ))
        J = np.linalg.norm(v-v_hat)
        if J < J_opt :
            J_opt = J
            tau_opt = tau_test
    return(tau_opt)

def characterize_routine(T):
    global velR_ave
    global velL_ave
    global vel_ctr
    
    rospy.init_node('characterization')

    # initialize message topics
    car_pub = rospy.Publisher('car_command',CarCommand,queue_size = 5)
    pwm_pub = rospy.Publisher('setVel',HighLevel,queue_size = 1)
    master_pub = rospy.Publisher('/master',UserCommand,queue_size = 5)
    rospy.Subscriber('tm4c',SensorData,callback)

    # T is the time in seconds for each test point.
    #r = rospy.Rate(T/2.0) # hz. 5 seconds

    N_TP = 3; # number of test points (length of _TF)
    left_TF = [0.5, 0.3, 0.4]
    right_TF =[0.5, 0.3, 0.4]
    left_Tv = [0] * N_TP
    right_Tv = [0] * N_TP
    
    N_TP2 = 4;
    ref_time = [0.3, 0.5, 0.8, 1.2]
    ref_time_diff = np.append(ref_time[0], np.diff(ref_time))
    left_Tvv = [0] * N_TP2
    right_Tvv = [0] * N_TP2

    Wp = 5 # averaging window size for each data point
    Ts = 1./20. # sample time; might want to set this directly from a ros variable...
    done = False


    pwm_msg = HighLevel()
    command_msg = CarCommand()
    
    rospy.sleep(1)
    command_msg.header.stamp = rospy.Time()
    command_msg.LL_active = False
    command_msg.PWM_mode = True
    command_msg.HL_active = False
    master_msg = UserCommand()
    master_msg.header.stamp = command_msg.header.stamp
    master_msg.master_on = True

    for i in range(0,4):
      car_pub.publish(command_msg)
      master_pub.publish(master_msg)

    rospy.sleep(1)
    # run and get data
    rospy.loginfo('Starting test mode')

    for i in range(0,N_TP):

        # set pwm through message
        pwm_msg.header.stamp = rospy.Time()
        (pwm_msg.pwmL,pwm_msg.pwmR) = (left_TF[i], right_TF[i])
        pwm_pub.publish(pwm_msg)

        # get some transient data from the first section for time constant
        if i == 0 :
            for j in range(0,N_TP2):
                rospy.sleep(ref_time_diff[j]-Ts)

                # reset velocity counter and get data after only one Ts (one sample)
                (velR_ave, velL_ave, vel_ctr) = (0.0,0.0,0)
                while not vel_ctr:
                  rospy.sleep(Ts)
                  
                left_Tvv[j]  = velL_ave / vel_ctr
                right_Tvv[j] = velR_ave / vel_ctr

            rospy.sleep(T- Ts*Wp - ref_time[N_TP2-1] ) # after transient data is done, wait for ss data
        else:
            # wait some time to enter steady state
            rospy.sleep(T - Ts*Wp)

        
        rospy.loginfo('Finished transient')

        # reset velocity counter
        (velR_ave, velL_ave, vel_ctr) = (0.0,0.0,0)
        while vel_ctr < 5:
           rospy.sleep(Ts)
        left_Tv[i]  = velL_ave / vel_ctr
        right_Tv[i] = velR_ave / vel_ctr

        # average left and right velocities over a small window in steady state
        rospy.loginfo([left_Tv[i],velL_ave,vel_ctr, right_Tv[i],velR_ave,vel_ctr])

    # stop car
    pwm_msg.header.stamp = rospy.Time.now()
    (pwm_msg.pwmL,pwm_msg.pwmR) = (0.0,0.0)
    pwm_pub.publish(pwm_msg)
    master_msg.header.stamp = pwm_msg.header.stamp
    master_msg.master_on = False 
    master_pub.publish(master_msg)

    (velR_ave, velL_ave, vel_ctr) = (0.0,0.0,0)
    
    rospy.loginfo('Testing finished. Now finding motor constants.')

    rospy.loginfo([left_Tv,right_Tv])

    # perform SVD + char, left and right motors
    (deadbandL,KL) = find_dbK(left_TF,left_Tv,N_TP)
    (deadbandR,KR) = find_dbK(right_TF,right_Tv,N_TP)

    tauL = find_tau(ref_time, left_Tvv, deadbandL, KL, left_TF[0])
    tauR = find_tau(ref_time, right_Tvv, deadbandR, KR, right_TF[0])
    rospy.loginfo([left_Tvv,right_Tvv,KL,KR])
    
    (aL,aR,bL,bR) = (KL/tauL,KR/tauR,-1./tauL,-1./tauR)
    rospy.set_param('motor_gains/alphaL', aL.item())
    rospy.set_param('motor_gains/alphaR', aR.item())
    rospy.set_param('motor_gains/betaL', bL.item())
    rospy.set_param('motor_gains/betaR', bR.item())
    rospy.set_param('motor_gains/deadbandL', deadbandL.item())
    rospy.set_param('motor_gains/deadbandR', deadbandR.item())
    setGlobals()

    log_string = 'The motor gains for' +\
            rospy.get_namespace() +\
            'are: \n' \
            '\nalphaL: ' +\
            str(KL/tauL) +\
            '\nalphaR: ' +\
            str(KR/tauR) +\
            '\nbetaL: ' +\
            str(-1/tauL) +\
            '\nbetaR: ' +\
            str(-1/tauR) +\
            '\ndeadbandL: ' +\
            str(deadbandL) +\
            '\ndeadbandR: ' +\
            str(deadbandR)
    rospy.loginfo(log_string)

    rospy.spin()

def setGlobals(): 
    global alphaL  
    global alphaR   
    global betaL     
    global betaR      
    global dL        
    global dR         
    global deadbandL   
    global deadbandR   

    alphaL      = rospy.get_param('motor_gains/alphaL', -1)
    alphaR      = rospy.get_param('motor_gains/alphaR', -1)
    betaL       = rospy.get_param('motor_gains/betaL', -1)
    betaR       = rospy.get_param('motor_gains/betaR', -1)
    dL          = rospy.get_param('motor_gains/dL', -1)
    dR          = rospy.get_param('motor_gains/dR', -1)
    deadbandL   = rospy.get_param('motor_gains/deadbandL', -1)
    deadbandR   = rospy.get_param('motor_gains/deadbandR', -1)

def callback(data):
    global velR_ave
    global velL_ave
    global vel_ctr
    velL_ave += data.left.velocity
    velR_ave += data.right.velocity
    vel_ctr += 1

if __name__=='__main__':
    try:
        characterize_routine(4)
    except rospy.ROSInterruptException:
        pass

