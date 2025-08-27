
#import numpy as np
import threading, time
import cv2
import sys, os
import argparse, json

# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from rm_arm.core.rm_robot_controller import RobotArmController
from utils.data_io import DataRecorder

#from env_cfg import *

# Realman robot version grasp motion

def init_vice_arm(use_real_robot=False):
    robot = RobotArmController("192.168.1.102", 8080, 3)
    work_frmae_ret = robot.robot.rm_change_work_frame("Base")
    robot.get_arm_software_info()
    return 0



def goto_undertake_pose():
    #waiting for change
    q_undertake_pose=[95.2170, 57.6819, -113.0309, -9.6590, -63.36500, 176.8399]
    robot.moveJ(q_undertake_pose)
    return 0


def execute_dump_action():
    #waiting for change
    q_sump_pose=[95.2170, 57.6819, -113.0309, -9.6590, -63.36500, 176.8399]
    robot.moveJ(q_dump_pose)
    return 0
    

if __name__ == "__main__":  
    init_ret = init_vice_arm()

    # move to initial pose   
    # need change to new pose
    q_init = [95.2170, 57.6819, -113.0309, -9.6590, -63.36500, 176.8399]#sensing pose 
    if init_ret==0:
        robot.moveJ(q_init)

    ## ================= Global Variables =================
    # RML63 vice-arm control stuff
    q_curr, _ = robot.get_current_state()
    JMOVE_STEP = 0.005
    q_target = q_init
    marching_dist = 0.
    JLIMIT = 3.1

    ## ================= Global Variables =================


    try:
        while True:
            #

            if key & 0xFF == ord('q') or key == 27: # Press esc or 'q' to close the image window
                print("\n===== quit =====")  
                break

            elif key & 0xFF == ord('u'):
                goto_undertake_pose()
                print("\n=goto undetake=")
                continue

            elif key & 0xFF == ord('d'):
                execute_dump_action()
                print("\n=goto dump=")
                continue


      
    finally:
        if robot:
            robot.disconnect()  
            print("Program terminated.")
