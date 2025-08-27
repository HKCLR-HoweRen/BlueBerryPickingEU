from eu_arm.robot_arm_interface import RobotArm
from utils.data_io import DataRecorder


def init()
    

if __name__ == "__main__":  
     robot, pump_ctrl, rs_ctrl, grasp_algo = init_all()

    # move to initial pose            
    q_init = sensing_pose_list[sensing_pose_idx]
    robot.moveJ(q_init)

    # Visualization
    cv2.namedWindow('test_grasping', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('test_grasping', int(1280), int(720))
    cv2.setMouseCallback("test_grasping", click_callback)

    ## ================= Global Variables =================
    # EU arm control stuff
    q_curr, _ = robot.get_current_state()
    j_idx = 0
    JMOVE_STEP = 0.005
    q_target = q_init
    PRE_GRASP_DIST = 0.05 - 0.005 # mm
    GRIPPER_LENGTH = 0.135 # mm
    GRASP_OFFSET = 0.015  
    saving_data_idx = 0
    refPt = (int(1280/2), int(720/2)) # center of img
    refPt_updated = False
    marching_dist = 0.
    JLIMIT = 3.1
    ## ================= Global Variables =================

    data_io = DataRecorder('tmp/picking_data')

    try:
        while True: