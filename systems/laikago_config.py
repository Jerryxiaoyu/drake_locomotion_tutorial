
import numpy as np
import pydrake
import os
import pydrake

# model path
MODEL_PATH_ROOT = '/home/drl/PycharmProjects/underacuatedRoboics/model_world'

cinder_block_wide_sdf_path = os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/cinder_block_wide/model.sdf")
laikago_urdf_path = os.path.join(MODEL_PATH_ROOT, 'laikago_v1/laikago2.1.urdf')
ground_urdf_path = os.path.join(MODEL_PATH_ROOT, 'world/plane.urdf')

JointName_list = [
    "FL_hip_joint", "FR_hip_joint", "RR_hip_joint", "RL_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RR_thigh_joint", "RL_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RR_calf_joint", "RL_calf_joint",
]
ActuatorName_list = [
    'FL_hip_motor', 'FL_thigh_motor', 'FL_calf_motor',
    'FR_hip_motor', 'FR_thigh_motor', 'FR_calf_motor',
    'RR_hip_motor', 'RR_thigh_motor', 'RR_calf_motor',
    'RL_hip_motor', 'RL_thigh_motor', 'RL_calf_motor']

Leg_list = ['FL', 'FR', 'RR', 'RL']

MAX_TORQUE = 50

ROBOT_STANCE_CONFIGURATION =  np.array([0, 0  , 0, 0, 0.62, 0.62, 0.62, 0.62, -1.06, -1.06, -1.06, -1.06])

ROBOT_TRI_STANCE_CONFIGURATION = np.array([0, 0.26 , 0, 0, -0.2, 0.18, 0.62, 0.62, -0.85, -0.85, -0.85, -0.85])

ACTUATOR_CHOICE_MATRIX = np.array(
    [[1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.],
     [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.],
     [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.],
     [0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.],
     [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.]])