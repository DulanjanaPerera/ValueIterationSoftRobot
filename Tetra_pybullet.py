import pybullet as p
import pybullet_data  # this apparently contains the plane and r2d2
import math
import numpy as np
from numpy import linalg as lin


class TetraSim():
    def __init__(self, gui: bool = False, force: int = 100) -> None:
        """
        Initialize the simulation parameters
        :param gui: Graphical interface or simulation engine
        :param force: The torque
        """

        self.gui = gui
        self.ur = None
        self.step_finished = False
        self.action_done = False
        self.followcam = False
        self.maxForce = force
        self.zoom = 2

        self.xmin = -0.2
        self.xmax = 1.6
        self.ymin = -0.2
        self.ymax = 0.6
        self.dxy = 0.2

        self.x_states = None
        self.y_states = None
        self.o_states = None
        self.states = None
        self.actions = ['r', 'b', 'g', 'y']

        self.discretization()

        self.StartPos = [0, 0, 0.5]
        self.StartOrientation = p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2])


    def getPosition(self):
        if self.step_finished:
            return np.round(p.getLinkState(self.ur, 0)[0], 2)
        else:
            return np.array([0.0, 0.0, 0.0])

    def getFullPosition(self):
        if self.step_finished:
            xy1 = p.getLinkState(self.ur, 0)[0]
            xy2 = p.getLinkState(self.ur, 15)[0]
            xy3 = p.getLinkState(self.ur, 31)[0]
            xy4 = p.getLinkState(self.ur, 47)[0]

            return np.round([xy1[0], xy1[1], xy2[0], xy2[1], xy3[0], xy3[1], xy4[0], xy4[1]], 2)
        else:
            return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def getHeadPosition(self):  # get z-coordinates of tip
        if self.step_finished:
            z1 = p.getLinkState(self.ur, 15)[0]  # Blue
            z2 = p.getLinkState(self.ur, 32)[0]  # Yellow
            z3 = p.getLinkState(self.ur, 49)[0]  # Green
            z4 = p.getLinkState(self.ur, 65)[0]  # Red

            return np.round([z1[2], z2[2], z3[2], z4[2]], 2)  # [B Y G R]
        else:
            return np.array([0.0, 0.0, 0.0, 0.0])

    def getHead(self, index: bool = False):
        if self.step_finished:
            z = list(self.getHeadPosition())
            if ~index:
                if z[0] > 1:
                    return 'b'
                elif z[1] > 1:
                    return 'y'
                elif z[2] > 1:
                    return 'g'
                elif z[3] > 1:
                    return 'r'
            else:
                if z[0] > 1:
                    return 0
                elif z[1] > 1:
                    return 1
                elif z[2] > 1:
                    return 2
                elif z[3] > 1:
                    return 3
        else:
            return -1

    def is_actionDone(self):
        vel_array = self.getRobotBaseVelocity()
        vel = round(lin.norm(vel_array), 2)
        if abs(vel) < 0.05:
            self.action_done = True
            return True
        else:
            self.action_done = False
            return False

    def unit_v(self, v):
        return v / np.linalg.norm(v)

    def blueLegAngle(self):
        if self.step_finished:
            blue = np.array(np.round(p.getLinkState(self.ur, 15)[0], 2))
            base = np.array(self.getPosition())
            flagH = self.getHead()
            if flagH != "blue":
                vec = blue[:2] - base[:2]
                vec = self.unit_v(vec)
                x_vec = self.unit_v(np.array([1, 0]))
                if vec[-1] < 0:
                    return -1 * np.arccos(np.clip(np.dot(x_vec, vec), -1.0, 1.0))
                else:
                    return np.arccos(np.clip(np.dot(x_vec, vec), -1.0, 1.0))
            else:
                return -10

    def redLegAngle(self):
        if self.step_finished:
            blue = np.array(np.round(p.getLinkState(self.ur, 65)[0], 2))
            base = np.array(self.getPosition())
            flagH = self.getHead()
            if flagH != "red":
                vec = blue[:2] - base[:2]
                vec = self.unit_v(vec)
                x_vec = self.unit_v(np.array([1, 0]))
                if vec[-1] < 0:
                    return -1 * np.arccos(np.clip(np.dot(x_vec, vec), -1.0, 1.0))
                else:
                    return np.arccos(np.clip(np.dot(x_vec, vec), -1.0, 1.0))
            else:
                return -10

    def getRobotBasePosition(self):  # robot base cartesian location
        if self.step_finished:
            # return np.round(p.getLinkState(self.ur, 0)[4], 2)
            pos, ori = p.getBasePositionAndOrientation(self.ur)
            factor = round(1/self.dxy)
            pos = np.round(np.array(pos)*factor)/factor  # to round off to closest 0.05

            for i in range(0, 3):
                if pos[i] == -0.0:
                    pos[i] = 0.0

            return pos
        else:
            return np.array([0.0, 0.0, 0.0])

    def getRobotBaseVelocity(self):  # robot base linear velocity
        if self.step_finished:
            return np.round(p.getBaseVelocity(self.ur)[0], 2)
        else:
            return np.array([0.0, 0.0, 0.0])

    def getRobotBaseOrientation(self):  # robot base orientation
        if self.step_finished:
            pos, ori = p.getBasePositionAndOrientation(self.ur)
            orientation = np.round(p.getEulerFromQuaternion(ori), 2)
            return np.array(orientation)
        else:
            return np.array([0, 0, 0])

    def getHeadingError(self, curr_head):  # this function is mainly for pi heading direction
        if curr_head > 0:
            return np.round(-curr_head, 2)
        else:
            return np.round(curr_head, 2)

    def getGoalError(self):
        curr_pos = self.getRobotBasePosition()
        des_pos = self.goal
        mse = np.square(np.subtract(des_pos, curr_pos)).mean()
        return np.round(mse, 2)

    def getLegStates(self) -> list:
        joint_indices = np.arange(1, 67).tolist()
        joint_s = list(p.getJointStates(self.ur, joint_indices))
        joint_pos = []
        for s in joint_s:
            joint_pos.append(s[0])
        return joint_pos

    def getRewards(self, heading_error):
        # reward for keep the heading direction
        headErrorThreshold = abs(heading_error)
        if headErrorThreshold < 0.08:
            lambda_error = 2
        else:
            lambda_error = -1

        # reward for moving goal direction
        velocity = -self.getRobotBaseVelocity()[0]
        if velocity > 0.7:
            lambda_vel = 5
        else:
            lambda_vel = -2

        # Reward for distance traveled in the step
        distance = (self.prev_distance - self.getRobotBasePosition()[0])
        lambda_dis = distance * 10
        self.prev_distance = self.getRobotBasePosition()[0]

        # reward for reaching the goal
        goal_error = self.getGoalError()
        if abs(goal_error) < 0.5:
            lambda_goal = 100
        else:
            lambda_goal = -5

        return lambda_error + lambda_vel + lambda_dis + lambda_goal

    def OrinetationIndex(self):
        ori = self.getRobotBaseOrientation()
        ind = 0
        for o in self.o_states:
            dif = abs(o-ori)
            count = 0
            for i in dif:
                if i < 0.1:
                    count = count + 1
            if count == 3:
                return ind
            ind = ind + 1
        return -1
    def setEnv(self, pos=None, ori=None):
        self.step_finished = False
        if pos is None:
            pos = [0, 0, 0.5]
        if ori is None:
            orientation = p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2])
        else:
            orientation = p.getQuaternionFromEuler(ori)

        p.resetBasePositionAndOrientation(self.ur, pos, orientation)
        joint_list = np.arange(1, 67).tolist()
        for idx in joint_list:
            p.resetJointState(self.ur, idx, targetValue=0.0, targetVelocity=0.0)
        while (1):
            maxForce = np.ones(8) * self.maxForce
            [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir] = [0, 0, 0, 0, 0, 0, 0, 0]

            # position values
            positionVal = [math.cos(s_dir) * -s_str,
                           math.sin(s_dir) * -s_str,
                           math.cos(fr_dir) * -fr_str,
                           math.sin(fr_dir) * -fr_str,
                           math.cos(fl_dir) * -fl_str,
                           math.sin(fl_dir) * -fl_str,
                           math.cos(rr_dir) * -rr_str,
                           math.sin(rr_dir) * -rr_str]
            # print(self.ur)
            ## Blue
            p.setJointMotorControlArray(self.ur, np.arange(1, 16, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[0], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(2, 17, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[1], forces=maxForce)

            ## Yellow
            p.setJointMotorControlArray(self.ur, np.arange(18, 33, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[2], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(19, 34, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[3], forces=maxForce)

            ## Green
            p.setJointMotorControlArray(self.ur, np.arange(35, 50, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[4], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(36, 51, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[5], forces=maxForce)

            ## Red
            p.setJointMotorControlArray(self.ur, np.arange(51, 66, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[6], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(52, 67, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[7], forces=maxForce)
            p.stepSimulation()

            if self.getPosition().tolist()[2] < 0.4:
                break
        self.step_finished = True

    def resetEnv(self):
        self.step_finished = False
        pos = [0, 0, 0.5]
        ori = p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2])

        p.resetBasePositionAndOrientation(self.ur, pos, ori)
        joint_list = np.arange(1, 67).tolist()
        for idx in joint_list:
            p.resetJointState(self.ur, idx, targetValue=0.0, targetVelocity=0.0)
        while (1):
            maxForce = np.ones(8) * self.maxForce
            [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir] = [0, 0, 0, 0, 0, 0, 0, 0]

            # position values
            positionVal = [math.cos(s_dir) * -s_str,
                           math.sin(s_dir) * -s_str,
                           math.cos(fr_dir) * -fr_str,
                           math.sin(fr_dir) * -fr_str,
                           math.cos(fl_dir) * -fl_str,
                           math.sin(fl_dir) * -fl_str,
                           math.cos(rr_dir) * -rr_str,
                           math.sin(rr_dir) * -rr_str]
            # print(self.ur)
            ## Blue
            p.setJointMotorControlArray(self.ur, np.arange(1, 16, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[0], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(2, 17, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[1], forces=maxForce)

            ## Yellow
            p.setJointMotorControlArray(self.ur, np.arange(18, 33, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[2], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(19, 34, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[3], forces=maxForce)

            ## Green
            p.setJointMotorControlArray(self.ur, np.arange(35, 50, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[4], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(36, 51, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[5], forces=maxForce)

            ## Red
            p.setJointMotorControlArray(self.ur, np.arange(51, 66, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[6], forces=maxForce)
            p.setJointMotorControlArray(self.ur, np.arange(52, 67, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                        targetPositions=np.ones(8) * positionVal[7], forces=maxForce)
            p.stepSimulation()

            if self.getPosition().tolist()[2] < 0.35:
                break
        self.step_finished = True

    def getTopple(self):
        [ox, oy, oz] = self.getRobotBaseOrientation()
        if ox > 1.1 or oy < 0.2:
            return True
        else:
            return False

    def getTermination(self):
        if self.getGoalError() < 0.5:
            return True
        elif self.getTopple():
            return True
        else:
            return False

    def endSimulation(self):
        print("Ending Simulation...")
        p.disconnect()

    def run(self):

        if self.gui:
            print("Starting Simulation...")
            p.connect(p.GUI)
            self.followcam = True
        else:
            p.connect(p.DIRECT)
            self.followcam = False
        p.resetSimulation()
        if self.followcam:
            p.resetDebugVisualizerCamera(cameraDistance=self.zoom, cameraYaw=0, cameraPitch=-45,
                                         cameraTargetPosition=(0, 0, 1))
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0)
        planeId = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -20)
        p.changeDynamics(planeId, -1, lateralFriction=1)

        ## Add the goal point
        p.addUserDebugPoints([[1.2, 0.0, 0.2]], [[1, 0, 0]], pointSize=10, lifeTime=0)


        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        self.ur = p.loadURDF("./tetra.urdf", self.StartPos, self.StartOrientation, flags=p.URDF_USE_SELF_COLLISION)

    def step(self, action: list) -> None:
        """
        action is a list
        [back leg, right leg, lef leg, top leg]
        action = [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir]
        """
        self.step_finished = False

        maxForce = np.ones(8) * self.maxForce

        [s_str, s_dir, fr_str, fr_dir, fl_str, fl_dir, rr_str, rr_dir] = action
        bending = [s_str, fr_str, fl_str, rr_str]
        [s_str, fr_str, fl_str, rr_str] = np.clip(bending, -math.pi / 8, math.pi / 8)

        positionVal = [math.cos(s_dir) * -s_str,
                       math.sin(s_dir) * -s_str,
                       math.cos(fr_dir) * -fr_str,
                       math.sin(fr_dir) * -fr_str,
                       math.cos(fl_dir) * -fl_str,
                       math.sin(fl_dir) * -fl_str,
                       math.cos(rr_dir) * -rr_str,
                       math.sin(rr_dir) * -rr_str]
        p.setJointMotorControlArray(self.ur, np.arange(1, 16, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[0], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(2, 17, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[1], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(18, 33, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[2], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(19, 34, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[3], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(35, 50, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[4], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(36, 51, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[5], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(51, 66, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[6], forces=maxForce)
        p.setJointMotorControlArray(self.ur, np.arange(52, 67, 2).tolist(), controlMode=p.POSITION_CONTROL,
                                    targetPositions=np.ones(8) * positionVal[7], forces=maxForce)

        p.stepSimulation()
        self.step_finished = True

    def discretization(self) -> None:
        self.x_states = np.round(np.arange(self.xmin, self.xmax + self.dxy, self.dxy), 2)
        self.y_states = np.round(np.arange(self.ymin, self.ymax + self.dxy, self.dxy), 2)

        dt = 1.2316
        pi = math.pi
        self.o_states = np.array([[pi, 0, -pi / 2],
                                  [-dt, 0.49, -0.87],
                                  [1.18, 0.49, -2.29],
                                  [0, -dt, -pi/2],
                                  [0, -dt, pi / 2],
                                  [-1.18, 0.49, 2.29],
                                  [1.18, 0.49, 0.87],
                                  [-pi, 0, pi / 2]])
