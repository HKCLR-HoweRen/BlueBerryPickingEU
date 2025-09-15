import numpy as np
import open3d as o3d
import cv2
import threading, time
import sys, os
import argparse, json
import asyncio
import websockets
from websockets.exceptions import ConnectionClosedError

# Add the parent directory of src to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from dev.rs_d405 import RealSenseController
from dev.pump_control import PumpControl
from rm_arm.core.rm_robot_controller import RobotArmController
from sensing_algo.grasp_algo import GraspingAlgo

from utils.data_io import DataRecorder

np.set_printoptions(precision=7, suppress=True)

# Realman robot version grasp motion
def async_grasp_v2(marching_dist):
    global pump_ctrl, GRASP_OFFSET
    print("===== stepping forward =====")  
    fwd_motion = np.eye(4)
    fwd_motion[2, 3] = marching_dist + GRASP_OFFSET
    robot1.moveL_relative_v2(fwd_motion, v=30)

    print("===== closing gripper =====")  
    closeGripper(pump_ctrl)

    print("===== rotating =====")  
    q_target_inc = [0, 0, 0, 0, 0, 90 / 180 * np.pi] # 90deg for last joint
    robot1.moveJ_relative(q_target_inc, 50, 1)

    print("===== getting back =====")  
    bwd_motion = np.eye(4)
    bwd_motion[2, 3] = -0.1
    robot1.moveL_relative_v2(bwd_motion, v=30)
    print('async grasping done')
    return

def infer(algo, img):
    return algo.infer_img(img)

def closeGripper(pump_ctrl):
    if pump_ctrl is not None:
        return pump_ctrl.config_gripper(1)
    time.sleep(0.5)
    return True

def releaseGripper(pump_ctrl):
    if pump_ctrl is not None:
        return pump_ctrl.config_gripper(0)
    time.sleep(0.5)
    return True

def validate_grasp_pose(pose):
    x, y, z = pose[0, 3], pose[1, 3], pose[2, 3]
    x_ok = abs(x) < 0.25
    y_ok = y > -0.4 and y < -0.1
    z_ok = z > -0.2 and z < 0.55
    print(f'validating pos [{x:.4f}, {y:.4f}, {z:.4f}]', [{x_ok}, {y_ok}, {z_ok}])
    return x_ok and y_ok and z_ok

def click_callback(event, x, y, flags, param):
    # grab references to the global variables
    global refPt, refPt_updated
        
    if event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        refPt_updated = True

def init_all(use_real_robot=True, use_pump=True, fake_camera=False, checkpoint_path=None):
    robot1 = RobotArmController("192.168.123.172", 8080, 3, 2)
    # robot2 = RobotArmController("192.168.123.172", 8080, 3)
    robot2 = None
    
    # TODO: set robot2  
    
    ret = robot1.robot.rm_change_work_frame("Base")
    robot1.get_arm_software_info()
    
    # Pump init
    if use_pump:
        pump_ctrl = PumpControl()
    else:
        pump_ctrl = None
        
    # Realsense init
    import ipdb; ipdb.set_trace()
    rs_ctrl = RealSenseController()
    rs_ctrl.config_stereo_stream()
    
    # Vision Algo
    grasp_algo = GraspingAlgo(checkpoint_path)

    return robot1, robot2, pump_ctrl, rs_ctrl, grasp_algo

class HiddenBlackboard():
    def __init__(self):
         pass

class WebSocketClient:
    def __init__(self, server_uri):
        self.server_uri = server_uri
        self.websocket = None
        self.connected = False
        print(f"WebSocket客户端已创建，目标服务器: {server_uri}")

    async def connect(self):
        try:
            self.websocket = await websockets.connect(self.server_uri)
            self.connected = True
            print("已连接到WebSocket服务器")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    async def send_data(self, message):
        if not self.connected or self.websocket is None:
            print("未连接到服务器，无法发送数据")
            return False
            
        try:
            await self.websocket.send(message)
            print(f"消息已发送: {message}")
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            await self.close()
            return False

    async def receive_data(self):
        if not self.connected or self.websocket is None:
            print("未连接到服务器，无法接收数据")
            return None
            
        try:
            data = await self.websocket.recv()
            if data:
                print(f"从服务器接收: {data}")
                return data
            else:
                print("连接已关闭")
                await self.close()
                return None
        except Exception as e:
            print(f"接收失败: {e}")
            await self.close()
            return None

    async def close(self):
        if self.websocket:
            await self.websocket.close()
            self.connected = False
            print("WebSocket连接已关闭")

async def websocket_client_task():
    global _lck, _start_exec, _exec_done
    
    # client = WebSocketClient("ws://192.168.123.166:12000")
    client = WebSocketClient("ws://127.0.0.1:12000")
    
    if not await client.connect():
        print("无法连接到WebSocket服务器，websocket_client_task退出")
        return
        
    try:
        # 发送就绪消息
        # await client.send_data("Arm ready")
            
        # 主循环 - 持续监听服务器消息
        while True:
            # 接收数据
            data = await client.receive_data()
            if data is None:
                print("no data, waiting ...//连接断开，websocket_client_task退出")
                break
                
            # 设置执行标志
            with _lck:
                _start_exec = True
                _exec_done = False
                
            print(f"收到任务指令: {data}")
            
            # 等待执行完成
            task_complete = False
            while not task_complete:
                await asyncio.sleep(0.5)
                with _lck:
                    task_complete = _exec_done
            
            # 发送任务完成消息
            await client.send_data("task complete")
                    
    except Exception as e:
        print(f"websocket_client_task发生异常: {e}")
    finally:
        await client.close()
        print("websocket_client_task退出")

def run_websocket_client():
    asyncio.run(websocket_client_task())

if __name__ == "__main__":  
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", "-c", default='env_cfg.json', help="Path to the config file")
    args = parser.parse_args()

    # Load configuration from JSON file
    with open(args.config, 'r') as f:
        cfg = json.load(f)

    # Loading params
    H_handeye = np.array(cfg['H_handeye'])
    detected_obj_pose_viewpoint = np.array(cfg['detected_obj_pose_viewpoint'])
    sensing_pose_list = np.array(cfg['sensing_pose_list'])
    sensing_pose_idx = cfg['sensing_pose_idx']

    use_real_robot = cfg['use_real_robot']
    use_pump = cfg['use_pump']
    use_real_camera = cfg['use_real_camera']

    save_data_path = cfg["save_data_path"]
    checkpoint_path = cfg["checkpoint_path"]

    print(f'H_handeye: {H_handeye}')
    for i in range(len(sensing_pose_list)):
        print(f'[{i}]th pose: {sensing_pose_list[i]}')
    
    robot1, robot2, pump_ctrl, rs_ctrl, grasp_algo = init_all(use_real_robot,
                                                              use_pump,
                                                              use_real_camera,
                                                              checkpoint_path)

    # move to initial pose            
    q_init = sensing_pose_list[sensing_pose_idx]
    side_arm_q_init = [-1.056,104.073,-175.102,-3.011,-16.015,117.023]
    if use_real_robot:
        robot1.moveJ(q_init, v=20, block=False)
        # robot2.moveJ(side_arm_q_init, v=20, block=False, degrees=True)

    # Visualization
    cv2.namedWindow('test_grasping', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('test_grasping', int(1280), int(720))
    cv2.setMouseCallback("test_grasping", click_callback)

    ## ================= Global Variables =================
    # EU arm control stuff
    q_curr, _ = robot1.get_current_state(fmt='matrix')
    j_idx = 0
    JMOVE_STEP = 0.005
    q_target = q_init
    PRE_GRASP_DIST = 0.05 - 0.005 # mm
    GRIPPER_LENGTH = 0.145 # mm
    GRASP_OFFSET = 0.0375  
    saving_data_idx = 0
    refPt = (int(1280/2), int(720/2)) # center of img
    refPt_updated = False
    marching_dist = 0.
    JLIMIT = 3.1
    
    # WebSocket client
    _lck = threading.Lock()
    _start_exec = False
    _exec_done = False
    ## ================= Global Variables =================
    
    
    t_ws_client = threading.Thread(target=run_websocket_client, args=())
    t_ws_client.start()
    
    # task_count = 0
               
    data_io = DataRecorder('tmp/picking_data')
    try:
        while True:
            color_frame, depth_frame = rs_ctrl.get_stereo_frame()
            color_img = np.asanyarray(color_frame.get_data())

            # Visiualization and interactive control
            cv2.imshow('test_grasping', color_img)
            key = cv2.waitKey(1)
            
            #============mutex lock============
            _lck.acquire()
            start_exec = _start_exec
            if start_exec:
                # 重置执行标志
                _start_exec = False
                # 重置完成标志
                _exec_done = False
                task_count += 1
                print(f"\n===== 开始执行第 {task_count} 个任务 =====")
            _lck.release()
            #============mutex lock============
            
            if key & 0xFF == ord('q') or key == 27: # Press esc or 'q' to close the image window
                print("\n===== stop pipeline =====")  
                cv2.destroyAllWindows()
                break
            
            elif key & 0xFF == ord('s') or start_exec:
                print("\n===== start inference =====")
                pc = rs_ctrl.convert_to_pointcloud(color_frame, depth_frame)
                q_curr, sensing_pose_eef = robot1.get_current_state(fmt='matrix')
                data_io.save_data(color_img, pc, q_curr)
                refPt_updated = False

                # Segmentation
                masks, bboxes, _ = infer(grasp_algo, color_img)
                if masks is not None and len(masks) == 0:
                            print('>'*15, 'no object detected', '>'*15)
                            #============mutex lock============
                            _lck.acquire()
                            _start_exec = False
                            _lck.release()
                            #============mutex lock============
                            continue
                
                grasp_poses, grasp_uv = grasp_algo.gen_grasp_pose(color_img, pc, masks)
                img_vis = cv2.imread('/home/hkclr/AGBot_ws/V2/BlueBerryPickingEU/tmp/test_img_cups_.png') #TODO: fix abs path
                while not refPt_updated:
                    cv2.imshow('test_grasping', img_vis)
                    key = cv2.waitKey(500)
                    print('===== Click to select a grasp pose')

                if grasp_poses is None:
                    print('>'*15, 'Could not find valid grasp pose', '>'*15)
                    #============mutex lock============
                    _lck.acquire()
                    _start_exec = False
                    _lck.release()
                    #============mutex lock============
                    continue

                grasp_idx = GraspingAlgo.get_nearest_inst(grasp_uv, refPt)
                print(f'finding nearest inst from ref pt: {refPt}/[{grasp_uv[grasp_idx]}] [{grasp_idx}]->{grasp_poses[grasp_idx]}')    

                obj_pose_vec = grasp_poses[grasp_idx][0]
                # filter by robot workspace
                obj_pose = detected_obj_pose_viewpoint
                obj_pose[:3,3] = obj_pose_vec[:3]
                eef2obj = H_handeye @ obj_pose
                H_base_to_obj = sensing_pose_eef @ H_handeye @ obj_pose

                print(f"H_base_to_obj: {H_base_to_obj}")

                # TODO: adjust workspace limit by rm robot ws
                # if validate_grasp_pose(H_base_to_obj):
                #     detected_obj_pose_viewpoint[:3,3] = obj_pose_vec[:3]
                #     print(f'================detected pose: \n{repr(detected_obj_pose_viewpoint)}\n====================')  

                #============mutex lock============
                _lck.acquire()
                _start_exec = False
                _lck.release()
                #============mutex lock============

                continue

            elif key & 0xFF == ord('g'):
                print("\n go grasping")  
                q_curr, eef_pose = robot1.get_current_state(fmt='matrix')

                # pregrasp pose -> relative pose on x-y plane wrt eef
                eef2obj = H_handeye @ detected_obj_pose_viewpoint
                eef2pre_grasp= np.eye(4)                
                eef2pre_grasp[:3, 3] = eef2obj[:3, 3]
                # handle z offset
                if eef2obj[2, 3] > PRE_GRASP_DIST + GRIPPER_LENGTH:
                    marching_dist = PRE_GRASP_DIST
                    eef2pre_grasp[2, 3] = eef2obj[2, 3] - GRIPPER_LENGTH - PRE_GRASP_DIST
                else:
                    marching_dist = eef2obj[2, 3] - GRIPPER_LENGTH
                    eef2pre_grasp[2, 3] = 0
                print(f'marching_dist: {eef2obj[2, 3] - PRE_GRASP_DIST - GRIPPER_LENGTH} | pre_grasp_dist: [{eef2pre_grasp[2, 3]}]')
                print(f'eef2pre_grasp: \n{repr(eef2pre_grasp)}')

                H_base_to_obj = eef_pose @ eef2pre_grasp
                grasp_pose_tcp = H_base_to_obj

                print(f'eef_pose: \n{repr(eef_pose)}')
                print(f'H_base_to_obj: \n{repr(H_base_to_obj)}')
                print(f'grasp_pose: \n{repr(grasp_pose_tcp)}')

                robot1.move_to_pose(grasp_pose_tcp, v=30)
                continue

            elif key & 0xFF == ord('f'):
                print("\n ")  
                q_curr, eef_pose = robot1.get_current_state(fmt='matrix')

                # pregrasp pose -> relative pose on x-y plane wrt eef
                eef2obj = H_handeye @ detected_obj_pose_viewpoint
                eef2pre_grasp= np.eye(4)                
                eef2pre_grasp[:3, 3] = eef2obj[:3, 3]
                eef2pre_grasp[1, 3] -= 0.015
                # handle z offset
                if eef2obj[2, 3] > PRE_GRASP_DIST + GRIPPER_LENGTH:
                    eef2pre_grasp[2, 3] = eef2obj[2, 3] - GRIPPER_LENGTH - PRE_GRASP_DIST
                else:
                    eef2pre_grasp[2, 3] = 0

                H_base_to_obj = eef_pose @ eef2pre_grasp
                grasp_pose_tcp = H_base_to_obj
                robot1.move_to_pose(grasp_pose_tcp, v=10)
                continue

            elif key & 0xFF == ord('t'):
                t_grasp = threading.Thread(target=async_grasp_v2, args=(marching_dist,))
                t_grasp.start()
                continue

            elif key & 0xFF == ord('c'):
                sensing_pose_idx += 1
                idx = sensing_pose_idx % len(sensing_pose_list)
                sensing_pose = sensing_pose_list[idx]
                q_init = sensing_pose
                print(f'Moving to {idx+1}th sensing pose [{sensing_pose}]......\n')
                robot1.moveJ(sensing_pose, block=True)
                continue

            elif key & 0xFF == ord('v'):
                print("\n===== closing gripper =====")  
                closeGripper(pump_ctrl)
                continue

            elif key & 0xFF == ord('r'):
                print("\n===== releasing gripper =====")  
                releaseGripper(pump_ctrl)
                continue

            elif key & 0xFF == ord('i'):
                q_set = np.array([-1.18703, 0.64249, -2.27585, -0.02076, -0.42116, 3.04271])
                # robot2.moveJ(side_arm_q_init, v=20, block=False, degrees=True)
                robot1.moveJ(q_set)
                sensing_pose_idx = 0
                continue

            elif key & 0xFF == ord('p'):
                q_arm1_place = [-47.65299987792969, 24.76799964904785, -121.1259994506836, -1.1610000133514404, -62.19300079345703, 174.33599853515625]
                q_arm2_carry_obj = [70.20800018310547, 54.448001861572266, -160.46600341796875, -1.9869999885559082, 15.84000015258789, 117.02400207519531]
                robot1.moveJ(q_arm1_place, v=20, block=False, degrees=True)
                # robot2.moveJ(q_arm2_carry_obj, v=30, block=True, degrees=True)
                
                time.sleep(0.5)
                # TODO: wait for motion done, add trajectory execution monitor
                # import ipdb; ipdb.set_trace()
                
                q_arm2_place = [-4.380, 88.622,-155.187,-2.607,-36.824, 117.026]
                # robot2.moveJ(q_arm2_place, v=30, block=True, degrees=True)
                                
                q_target_inc = [0, 0, 0, 0, 0, -150 / 180 * np.pi] # 150 deg for last joint
                # robot2.moveJ_relative(q_target_inc, 50, 1)
                
                #============mutex lock============
                _lck.acquire()
                _exec_done = True
                _lck.release()
                #============mutex lock============
                
                continue

            elif key & 0xFF == ord('w'):
                q_target, _ = robot1.get_current_state(fmt='matrix')
                q_target[3] += JMOVE_STEP * 1.5
                robot1.moveJ(q_target)
                continue
            elif key & 0xFF == ord('e'):
                q_target, _ = robot1.get_current_state(fmt='matrix')
                q_target[3] -= JMOVE_STEP * 1.5
                robot1.moveJ(q_target)
                continue

            elif key & 0xFF == 82: # up
                q_target, _ = robot1.get_current_state(fmt='matrix')
                q_target[j_idx] += JMOVE_STEP * 1.5
                robot1.moveJ(q_target)
                continue
            elif key & 0xFF == 84: # down
                q_target, _ = robot1.get_current_state(fmt='matrix')
                q_target[j_idx] -= JMOVE_STEP * 1.5
                robot1.moveJ(q_target)
                continue
            elif key & 0xFF == 81: # left
                j_idx = (j_idx - 1) % 6 # 6 DOF
                print(f"\n===== switch to prev joint, Joint [{j_idx+1}] selected =====")  
                continue
            elif key & 0xFF == 83: # right
                j_idx = (j_idx + 1) % 6 # 6 DOF
                print(f"\n===== switch to next joint, Joint [{j_idx+1}] selected =====")  
                continue

    except:
        import ipdb; ipdb.set_trace()

    finally:
        rs_ctrl.stop_streaming()
        grasp_algo.free_cuda_buffers()
