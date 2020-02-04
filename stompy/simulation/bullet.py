
import os

import numpy
import pybullet
#import pybullet_data

from .. import signaler


global sim
sim = None


class Sim(signaler.Signaler):
    def __init__(self):
        super(Sim, self).__init__()
        self.conn = None
        self.legs = {}
        self.build()
        self.running = False
        #self.run()

    def register_leg(self, leg):
        pass

    def build(self):
        self.conn = pybullet.connect(pybullet.GUI)
        #pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        sd = os.path.join(os.path.dirname(__file__), 'data')
        pybullet.setAdditionalSearchPath(sd)
        pybullet.setGravity(0,0,-9.8)

        # TODO load terrain
        planeId = pybullet.loadURDF(
            "plane.urdf", [0, 0, 0], pybullet.getQuaternionFromEuler([-0.1, 0, 0]))
        dolliesId = pybullet.loadURDF("dollies.urdf")

        # load stompy
        self.robot_id = pybullet.loadURDF(
            "stompy.urdf", [0, 0, 2], pybullet.getQuaternionFromEuler([0, 0, 0]))

        # get joint ids
        nj = pybullet.getNumJoints(self.robot_id)
        self.legs = {}
        self.joint_ids = {}
        #print("N joints: %s" % (nj, ))
        for ji in range(nj):
            #print("Joint %i" % (ji, ))
            info = pybullet.getJointInfo(self.robot_id, ji)
            name = info[1].decode('ascii')
            ts = name.split('_')
            if ts[2] == 'body':  # hip joint
                ln = ts[4]
                jn = 'hip'
            else:
                ln = ts[2]
                if ts[-1] == 'thigh':
                    jn = 'thigh'
                elif ts[-1] == 'upper':
                    jn = 'knee'
                elif ts[-1] == 'lower':
                    jn = 'calf'
                else:
                    jn = None
            #print("\t%s" % (info, ))
            if jn is None:
                # make joint mostly passive
                #pybullet.setJointMotorControl2(
                #    self.robot_id, ji, pybullet.VELOCITY_CONTROL,
                #    force=4)
                pybullet.setJointMotorControl2(
                    self.robot_id, ji, pybullet.POSITION_CONTROL,
                    targetPosition=0.0, force=4.5)
                continue
            if ln not in self.legs:
                self.legs[ln] = {}
            self.legs[ln][jn] = ji
            self.joint_ids[ji] = {'leg': ln, 'joint': jn}
            if jn == 'calf':
                pybullet.setJointMotorControl2(
                    self.robot_id, ji, pybullet.POSITION_CONTROL,
                    targetPosition=0.0, force=5337)
            else:
                pybullet.setJointMotorControl2(
                    self.robot_id, ji, pybullet.POSITION_CONTROL,
                    targetPosition=0.0)

    def run(self):
        pybullet.setRealTimeSimulation(True)

    def set_joint_angles(self, angles):
        ja = []
        ji = []
        for ln in angles:
            for jn in angles[ln]:
                ji.append(self.legs[ln][jn])
                ja.append(angles[ln][jn])
        pybullet.setJointMotorControlArray(
            self.robot_id, ji, pybullet.POSITION_CONTROL, ja)

    def get_joint_states(self):
        states = {ln: {} for ln in self.legs.keys()}
        jids = list(self.joint_ids.keys())
        for (jid, state) in zip(
                jids, pybullet.getJointStates(self.robot_id, jids)):
            # pos, vel, reaction_force, applied_force
            joint_info = self.joint_ids[jid]
            ln = joint_info['leg']
            jn = joint_info['joint']
            if jn == 'calf':
                states[ln][jn] = state[-1]
            else:
                states[ln][jn] = state[0]
        self.trigger('joints', states)
        return states

    def get_orientation(self):
        _, oriq = pybullet.getBasePositionAndOrientation(self.robot_id)
        ori = [numpy.degrees(a) for a in pybullet.getEulerFromQuaternion(oriq)]
        ori[1] = -ori[1]
        self.trigger('orientation', ori)
        return ori

    def update(self):
        if not self.running:
            self.run()
            self.running = True
        # TODO throttle
        #self.get_joint_states()
        # update imu
        #self.get_orientation()

    def __del__(self):
        pybullet.disconnect()


def get():
    global sim
    if sim is None:
        sim = Sim()
    return sim
