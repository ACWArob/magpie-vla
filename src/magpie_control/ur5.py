########## INIT ####################################################################################

##### Imports ####################################
import time
from time import sleep
import asyncio
from concurrent.futures import ThreadPoolExecutor
from traceback import print_exc

# Numpy
import numpy as np
from numpy import radians

# Spatial Math is used for manipulating geometric primitives
import spatialmath as sm
from spatialmath import SE3

# UR Interface
import rtde_control
import rtde_receive
# from rtde_receive import RTDEReceiveInterface as RTDEReceive

# Gripper Interface
import serial.tools.list_ports
# from magpie.motor_code import Motors
from magpie_control.gripper import Gripper
from magpie_control.utils import transform_6d
# Poses is from rmlib and used for converting between 4 x 4 homogenous pose and 6 element vector representation (x,y,z,rx,ry,rz)
from magpie_control import poses

# FT Sensor
from magpie_control.ft_sensor import OptoForceCmd, OptoForce
from magpie_control.arm_utils import RunUntilAnyT, FTCondition

##### Constants ##################################
from magpie_control.homog_utils import homog_xform, R_krot

_CAMERA_XFORM = homog_xform( # TCP --to-> Camera is a rotation of -90 degrees about the z-axis
    rotnMatx = R_krot( [0.0, 0.0, 1.0], -np.pi/2.0 ),
    posnVctr = [0.0, 0.0,  0.120]
    # posnVctr = [-0.0173, -0.0018,  0.0183] # 2025-02-06: This was a terrible idea
)

########## HELPER FUNCTIONS ########################################################################


def pose_vector_to_homog_coord( poseVec ):
    """ Express the pose vector in homogeneous coordinates """
    # poseVector is a 6 element list of [x, y, z, rX, rY, rZ]
    return poses.pose_vec_to_mtrx( poseVec )


def homog_coord_to_pose_vector( poseMatrix ):
    """ Converts poseMatrix into a 6 element list of [x, y, z, rX, rY, rZ] """
    # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
    return poses.pose_mtrx_to_vec( np.array( poseMatrix ) )


def get_USB_port_with_desc( descStr ):
    """ Return the name of the first USB port that has `descStr` as a substring of its description, Otherwise return none """
    match = None
    for port, desc, hwid in sorted( serial.tools.list_ports.comports() ):
        if descStr in desc:
            match = port
            break
    return match


########## UR5 INTERFACE ###########################################################################
from typing import Callable

class UR5_Interface:
    """ Interface class to `ur_rtde` """

    @staticmethod
    def dummy_cb():
        return None


    def set_tcp_to_camera_xform( self, xform ):
        """ Set the camera transform """
        self.camXform = np.array( xform )


    def __init__( self, robotIP = "192.168.0.4", cameraXform = None, freq = 500, 
                 record=False, record_path=None, provide_gripper=False, provide_ft_sensor=False ):
        """ Store connection params and useful constants """
        self.name       = "UR5_CB3"
        self.robotIP    = robotIP # IP address of the robot
        self.ctrl       = None # -- `RTDEControlInterface` object
        self.recv       = None # -- `RTDEReceiveInterface` object
        self.gripper    = None # -- Gripper Controller Interface
        self.ft_sensor = None
        self.Q_safe     = [ radians( elem ) for elem in [ 34.65, -105.31, 101.03, 1.90, 38.37, 188.78 ] ]
        self.torqLim    = 600
        self.freq       = freq
        self.record     = record
        self.record_path = record_path
        self.gripClos_m = 0.002
        self.camXform   = np.eye(4)
        self.magpie_tooltip = [0.012, 0.006, 0.231] # wrist-relative xyz tooltip offset for magpie gripper
        self.debug      = False
        self.rerun_viz  = None
        self.home       = None # initialized in start()
        self.z_offset   = 0.02 # 2cm z offset for magpie gripper, just a tunable value to smooth things over
        self.provide_gripper = provide_gripper # disable gripper by default for separate control
        self.provide_ft_sensor = provide_ft_sensor
        self.cf_t, self.ft_t, self.robot_log = [], [], {}
        if cameraXform is None:
            self.set_tcp_to_camera_xform( _CAMERA_XFORM )
        else:
            self.set_tcp_to_camera_xform( cameraXform )
        self.move_cb : Callable = UR5_Interface.dummy_cb


    def set_move_callback( self, cbFunc : Callable ):
        """ This function should be called after every move """
        self.move_cb = cbFunc


    def start_gripper( self ):
        servoPort = get_USB_port_with_desc( "OpenRB" )
        if servoPort is not None:
            print( f"Found Dynamixel Port:\n{servoPort}\n" )
            # self.gripper = Motors( servoPort )
            # self.gripper.torquelimit( self.torqLim )
            self.gripper = Gripper( servoPort )
            print( "Connected to gripper!" )
            # self.gripper.set_torque( self.torqLim, finger='both')
        else:
            raise RuntimeError( "Could NOT connect to gripper Dynamixel board!" )


    def start_ft_sensor( self, ip_address: str = "192.168.0.5", port: int = 49152, poll_rate=50):
        self.ft_sensor = OptoForce(ip_address = ip_address, port = port, poll_rate = poll_rate)
        self.ft_sensor.connect()


    def reset_gripper_overload( self, restart = True ):
        """ Attempt to clear an overload error """
        if self.provide_gripper:
            self.gripper.reset_packet_overload()
            self.gripper.disconnect()
            if restart:
                sleep( 0.25 ) 
                self.start_gripper()
        else:
            print( "Gripper not connected, cannot reset overload" )


    def get_ft_data(self):
        return self.ft_sensor.recv_datum()


    def ft_control(self, qGoal, force, speed = 0.1):
        condition = FTCondition(self.ft_sensor, force)
        # nonblocking move
        self.moveJ(qGoal, rotSpeed = speed, asynch = True)
        self.threaded_conditional_stop(condition.cond)
        #TODO: development is here. Test if working?


    def get_control_update(self, cmd=np.zeros(6), ft_goal=np.zeros(6), 
                        ft_meas=np.zeros(6), p=0.0005, control_type="bang_bang"):
        if control_type == "bang_bang":
            sign = np.ones(6)
            sign[np.abs(ft_meas) > np.abs(ft_goal)] = -1
            # print(f"Flipping: {[axes[i] for i in range(6) if sign[i] == -1 and wrench[i] != 0]}")
            cmd = cmd * sign
            return cmd
        elif control_type == "proportional":
            update = p * (ft_goal - ft_meas)
            update[ft_goal == 0] = 0
            cmd_l = np.clip(cmd[:3] - update[:3], -0.09, 0.09)
            cmd_a = np.clip(cmd[3:] - update[3:], -0.04, 0.04) # lower limit for angular velocity
            cmd = np.hstack((cmd_l, cmd_a))
            return cmd
        return np.zeros(6)


    def force_position_control(self, wrench=np.zeros(6), init_cmd=np.zeros(6), 
                               goal_delta=[0,0,0], max_force=10, duration = 5, 
                               tolerance = 0.1, p=0.0005, control_type="bang_bang"):

        # self.gripper.cf_t, self.gripper.cf_t_ts, self.ft_t, self.robot_log, self.gripper.gripper_log = [], [], [], {}, {}
        self.ft_t, self.robot_log = [], {}
        pose, T_w  = np.array(self.getPose()), sm.SE3(goal_delta).A
        goal_pose = pose @ T_w
        distance = np.linalg.norm(goal_pose[:3, 3] - pose[:3, 3])
        ft_goal, ft_prev = np.array(wrench).clip(min=-1*max_force, max=max_force), self.get_ft_data()
        speedL_cmd_w = init_cmd # initial cmd
        start = time.time()
        if self.debug:
            print(f"Starting force-position control with goal delta {goal_delta}, max force {max_force} N")
            print(f"Initial Distance to goal: {distance:.4f} m")
            print(f"Initial FT: {ft_prev}, Goal FT: {ft_goal}")
            print(f"Initial Speed Command (wrist frame): {speedL_cmd_w}")

        # force control loop
        while time.time() - start < duration and distance > tolerance:
            if self.debug:
                print(f"Distance to goal: {distance:.4f} m")
                print(f"Current FT: {ft_prev}, Goal FT: {ft_goal}")
                print(f"Current Speed Command (wrist frame): {speedL_cmd_w}")
            self.speedL_TCP(np.array(speedL_cmd_w), tooltip=True)
            ft_curr = self.get_ft_data()
            if ft_curr == []: ft_curr = ft_prev # handle null reading from optoforce
            self.ft_t.append(ft_curr)

            timestamp = time.time()
            pose  = self.recv.getActualTCPPose()
            pose_mat = self.getPose().A
            speed = self.recv.getActualTCPSpeed()
            q     = self.recv.getActualQ()
            qd    = self.recv.getActualQd()
            action = {
                "actual_q": q,
                "actual_qd": qd,
                "actual_TCP_pose": pose,
                "actual_TCP_speed": speed,
                "actual_TCP_speed_wrist_frame": transform_6d(speed, pose_mat, pose_to_origin=False, is_wrench=False),
                "target_TCP_speed": speedL_cmd_w,
                "wrench": ft_curr,
                "wrench_base": transform_6d(ft_curr, pose_mat, pose_to_origin=True, is_wrench=True),
            }
            self.robot_log[timestamp] = action
            if self.rerun_viz is not None: self.rerun_viz.log_robot_data(action, timestamp, name="ur5")
            speedL_cmd_w = self.get_control_update(speedL_cmd_w, ft_goal, ft_curr, p=p, control_type=control_type)
            distance = np.linalg.norm(goal_pose[:3, 3] - np.array(self.getPose())[:3, 3])
            ft_prev = ft_curr

        self.ctrl.speedL([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5, 0.25)


    async def concurrent_gripper_camera_robot_control(self, wrench=np.zeros(6), grasp_force=2.0,
                                init_cmd=np.zeros(6), goal_delta=[0,0,0], max_force=10, 
                                duration = 5, tolerance = 0.1, p=0.0005, rerun_viz=None,
                                camera_dict={}, filepath="", control_type="bang_bang"):

        self.gripper.cf_t, self.gripper.cf_t_ts, self.ft_t, self.robot_log, self.gripper.gripper_log = [], [], [], {}, {}
        self.gripper.rerun_viz, self.rerun_viz = rerun_viz, rerun_viz
        if self.gripper is None:
            self.start_gripper()

        gripper_task = asyncio.create_task(self.gripper.reset_and_close_gripper_async(force_limit=grasp_force, 
                                                                                      duration=duration, record=True))
        if len(camera_dict) > 0:
            for camera in camera_dict:
                camera.rerun_viz = rerun_viz
                camera.begin_record(filepath=camera_dict[camera], record_depth=False)
        
        loop = asyncio.get_event_loop()
        with ThreadPoolExecutor() as pool:
            await loop.run_in_executor(pool, self.force_position_control, wrench, init_cmd, 
                                       goal_delta, max_force, duration, tolerance, p, control_type)

        await gripper_task
        if len(camera_dict) > 0:
            for camera in camera_dict:
                await camera.stop_record(write=False)
        
        motion_log = {"gripper": self.gripper.gripper_log, 
                      "robot": self.robot_log}
        for camera, filepath in camera_dict.items():
            motion_log[filepath] = camera.buffer_dict.copy()
            camera.buffer_dict = {}

        return motion_log


    def threaded_conditional_stop(self, condition = 'dummy'):
        def end_cond():
            nonlocal self
            #check if the robot speed is less than 1e-3
            return not self.p_moving()
        
        def stop_cb():
            nonlocal self
            self.ctrl.stopL()
        if condition == 'dummy':
            condition = lambda: False
        sched = RunUntilAnyT([condition, end_cond], stop_cb, 100)
        sleep(0.1)
        sched.run()


    def start( self ):
        """ Connect to RTDE and the gripper """
        # try:
        self.ctrl = rtde_control.RTDEControlInterface( self.robotIP )
        self.recv = rtde_receive.RTDEReceiveInterface( self.robotIP, self.freq )
        self.home = self.getPose()
        if self.provide_gripper: 
            self.start_gripper()
        if self.provide_ft_sensor: 
            self.start_ft_sensor()
        # except RuntimeError as e:
        #     print_exc()
        #     print( f"UR5 could not be started because --> {e}" )
            


    def halt( self ):
        """ I don't actually know if this is safe to do! """
        self.ctrl.servoStop()


    def revive(self):
        if not self.ctrl.isConnected():
            self.ctrl.reconnect()
            sleep(0.5)
        if not self.recv.isConnected():
            self.recv.reconnect()
            sleep(0.5)
        

    def stop( self ):
        """ Shutdown robot and gripper connections """
        self.ctrl.servoStop()
        self.ctrl.stopScript()
        if self.provide_gripper:
            self.gripper.disconnect()
        if self.ft_sensor is not None:
            self.ft_sensor.close()
        

    def get_name( self ):
        """ Get string that represents this robot """
        return self.name


    def get_joint_angles( self ):
        """ Returns a 6 element numpy array of joint angles (radians) """
        return np.array( self.recv.getActualQ() )


    def get_tcp_pose( self ) -> np.ndarray:
        """ Returns the current pose of the gripper as a SE3 Object (4 x 4 Homegenous Transform) """
        # return sm.SE3( pose_vector_to_homog_coord( self.recv.getActualTCPPose() ) )
        return pose_vector_to_homog_coord( self.recv.getActualTCPPose() )


    def getPose(self):
        # Returns the current pose of the last frame as a SE3 Object (4 x 4 Homegenous Transform)
        p = self.recv.getActualTCPPose()
        poseMatrix = self.poseVectorToMatrix(p)
        T_N = sm.SE3(poseMatrix)   # convert a pose vector to a matrix SE3 object, SE3 --> special euclidean in 3-dimensional space
        # T_N.plot(name="C")
        return T_N    # T_N is a homogenous transform


    def poseVectorToMatrix(self, poseVector):
        # Converts poseVector into an SE3 Object (4 x 4 Homegenous Transform)
        # poseVector is a 6 element list of [x, y, z, rX, rY, rZ]
        T_N = sm.SE3(poses.pose_vec_to_mtrx(poseVector))
        return T_N


    def get_cam_pose( self ) -> np.ndarray:
        """ Returns the current pose of the gripper as a SE3 Object (4 x 4 Homegenous Transform) """
        # return sm.SE3( pose_vector_to_homog_coord( self.recv.getActualTCPPose() ) )
        return np.dot(
            pose_vector_to_homog_coord( self.recv.getActualTCPPose() ),
            self.camXform
        )


    def get_sensor_pose_in_robot_frame( self, sensorPose ) -> np.ndarray:
        """ Get a pose obtained from segmentation in the robot frame """
        return np.dot(
            self.get_cam_pose(),
            sensorPose
            # sensorPose,
            # self.get_cam_pose()
        )


    def moveJ( self, qGoal, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ qGoal is a 6 element numpy array of joint angles (radians) """
        # speed is joint velocity (rad/s)
        if self.record:
            self.recv.startFileRecording(self.record_path, ["timestamp", "actual_q", "actual_TCP_pose"])
        self.ctrl.moveJ( list( qGoal ), rotSpeed, rotAccel, asynch )
        self.move_cb()


    def moveL( self, poseMatrix, linSpeed = 0.25, linAccel = 0.5, asynch = True):
        """ 
        Moves tool tip pose linearly in cartesian space to goal pose
        tool pose defined relative to the end of the gripper when closed
        poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
        """
        if self.record:
            self.recv.startFileRecording(self.record_path, ["timestamp", "actual_q", "actual_TCP_pose"])
        self.ctrl.moveL( homog_coord_to_pose_vector( poseMatrix ), linSpeed, linAccel, asynch )
        self.move_cb()


    def moveL_translation_tooltip( self, poseMatrix, TCP=np.zeros(3), z_offset=None, linSpeed = 0.25, linAccel = 0.5, asynch = True):
        """ 
        translates the tool tip pose in cartesian space to goal position (no orientation)
        standard moveL moves on the wrist frame
        """
        z_offset = self.z_offset if z_offset is None else z_offset
        tooltip = np.array(self.magpie_tooltip).copy() + np.array(TCP).copy()
        tooltip[2] -= z_offset
        p = poseMatrix[:3, 3] - np.array(tooltip)
        T = sm.SE3(p).A
        print(f"Tooltip: {tooltip}")
        goal = np.array(self.getPose()) @ T
        print(f"Goal Pos: {goal[:3, 3]}")
        self.moveL(goal, linSpeed, linAccel, asynch)
        self.move_cb()


    def speedL( self, speedL_cmd, linSpeed = 0.25, linAccel = 0.5 ):
        """ Set the linear speed and acceleration for the robot """
        if self.record:
            self.recv.startFileRecording(self.record_path, ["timestamp", "actual_q", "actual_qd", "actual_TCP_pose", "actual_TCP_speed"])
        self.ctrl.speedL( speedL_cmd, linSpeed, linAccel )


    def move_safe( self, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ Moves the arm linearly in joint space to home pose """
        self.moveJ( self.Q_safe, rotSpeed, rotAccel, asynch )
        self.move_cb()


    def moveL_delta(self, delta, frame="base", z_offset=0.0):
        '''
        moves the robot by a delta position (no orientation change)
        in either the base or wrist frame
        @param delta: relative position change along [x, y, z]
        @param frame: base or wrist frame along which to execute motion
        @param z_offset: offset in wrist_z, tunable constant to account for finicky TCP
        '''
        if frame not in ["base", "wrist"]:
            raise ValueError("frame must be either 'base' or 'wrist'")
        if frame=="wrist": delta[2] += z_offset
        T = sm.SE3(delta).A
        wrist = np.array(self.getPose()) 
        goal = None
        if frame=="wrist": # move w.r.t wrist frame
            goal = wrist @ T
        elif frame=="base": # move w.r.t base frame
            wrist[2, 3] += z_offset
            goal = T @ wrist
        self.moveL(goal)
        self.move_cb()


    async def moveL_delta_async(self, delta, frame="base", z_offset=0.0):
        '''
        moves the robot by a delta position (no orientation change)
        in either the base or wrist frame
        @param delta: relative position change along [x, y, z]
        @param frame: base or wrist frame along which to execute motion
        @param z_offset: offset in wrist_z, tunable constant to account for finicky TCP
        '''
        if frame not in ["base", "wrist"]:
            raise ValueError("frame must be either 'base' or 'wrist'")
        if frame=="wrist": delta[2] += z_offset
        T = sm.SE3(delta).A
        wrist = np.array(self.getPose()) 
        goal = None
        if frame=="wrist": # move w.r.t wrist frame
            goal = wrist @ T
        elif frame=="base": # move w.r.t base frame
            wrist[2, 3] += z_offset
            goal = T @ wrist
        self.moveL(goal)


    def speedL_TCP(self, wrist_speedL_cmd, linSpeed = 0.25, linAccel = 0.5, tooltip=False):
        '''
        moves the tool at some velocity in the wrist frame
        @param wrist_speedL_cmd: velocity command in wrist frame, [vx, vy, vz, wx, wy, wz]
        @param tooltip: if true, the speed command is in the tool frame, not wrist (only affects angular velocities)
        '''
        v_w = wrist_speedL_cmd[:3]
        w_w = wrist_speedL_cmd[3:]
        if tooltip:
            v_induced = np.cross(w_w, self.magpie_tooltip)
            v_w -= v_induced
        T = np.array(self.getPose())
        R = T[:3, :3]
        v_b = R @ v_w
        w_b = R @ w_w
        speedL_cmd = np.hstack((v_b, w_b))
        self.speedL(speedL_cmd, linSpeed, linAccel)


    def toggle_teach_mode(self):
        status = self.ctrl.getRobotStatus()
        if status == 7: # in teach mode
            self.ctrl.endTeachMode()
        elif status == 3: # regular control mode
            self.ctrl.teachMode()


    def stop_recording(self):
        if self.record:
            self.recv.stopFileRecording()


    def p_moving( self ):
        """ Return True if the robot is in motion, Otherwise return False """
        return not self.ctrl.isSteady()


    def open_gripper( self ):
        """ Open gripper to the fullest extent """
        # self.gripper.openGripper()
        self.gripper.open_gripper()


    def set_home( self, pose ):
        """ Set the home pose of the robot """
        # check that pose is a 4x4 homogeneous matrix
        if pose.shape != (4, 4):
            raise ValueError( "Pose must be a 4x4 homogeneous matrix" )
        self.home = pose


    def set_gripper( self, width, disable_wait = False ):
        """ Computes the servo angles needed for the jaws to be width mm apart """
        # Sends command over serial to the gripper to hold those angles
        # self.gripper.position( self.gripper.distance2theta( width) )
        self.gripper.set_goal_aperture( width, finger = 'both', debug = False, record_load = False, disable_wait = disable_wait )


    def set_grip_N( self, N ):
        """ Set the gripper fingers to N [N] """
        # self.set_gripper( self.gripClos_m )
        if 0.0 < N < 16.0:
            self.gripper.set_force( N, finger='both', debug=False)
        else:
            print( f"Force value {N} is out of range (0-16 N)" )


    def close_gripper( self ):
        """ Set the gripper fingers to near-zero gap """
        # self.set_gripper( self.gripClos_m )
        self.gripper.close_gripper()


    def get_gripper_sep( self ):
        """ Return the separation between the gripper fingers in [m] """
        return self.gripper.get_aperture( finger = 'both' ) / 1000.0


    # rotation matrix with -1, 1, -1 on diagonal
    def orient_TCP_down( self ):
        """orient TCP facing down toward the table, z-axis down"""
        R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
        p = np.array(self.getPose())
        p[:3, :3] = R
        self.moveL(p)
        self.move_cb()
        

    def align_tcp( self, lock_roll = False, lock_pitch = False, lock_yaw = False ):
        """
        Alignes the gripper with the nearest rotational axis (principle cartesian axes).
        Parameters
        ----------
        lock_roll:  bool
        lock_pitch: bool
        lock_yaw:   bool
        """
        pose       = self.get_tcp_pose()
        rot_matrix = pose[0:3, 0:3]
        R          = poses.rotation_mtrx_to_rpy( rot_matrix )
        for i, value in enumerate( R ):
            if i == 0 and lock_pitch:
                continue
            if i == 1 and lock_yaw:
                continue
            if i == 2 and lock_roll:
                continue
            if value > -3.142 and value <= -2.36:
                R[i] = -3.14  # -180
            elif value > -2.36 and value <= -0.79:
                R[i] = -1.57  # -90
            elif value > -0.79 and value <= 0.79:
                R[i] = 0  # 0
            elif value > 0.79 and value <= 2.36:
                R[i] = 1.57  # 90
            elif value > 2.36 and value <= 3.142:
                R[i] = 3.14  # 180
            else:
                raise RuntimeError("`align_tcp`: Encountered an unexpected value!")
        rot_matrix = poses.rpy_to_rotation_mtrx( R )
        pose[0:3, 0:3] = rot_matrix
        self.moveL( pose )
        self.move_cb()
        return pose