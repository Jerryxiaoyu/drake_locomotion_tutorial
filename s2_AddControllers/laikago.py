# -*- coding: utf8 -*-
#%%
import numpy as np
import pydrake
import os

from pydrake.common import AddResourceSearchPath, FindResourceOrThrow

from pydrake.multibody.tree import (UniformGravityFieldElement,
                                              WeldJoint,)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform



from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import SignalLogger

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from pydrake.systems.primitives import ConstantVectorSource, Sine
from pydrake.geometry import ConnectDrakeVisualizer

from pydrake.systems.controllers import InverseDynamicsController
from pydrake.attic.systems.controllers import RbtInverseDynamicsController
from pydrake.attic.multibody.rigid_body_tree import RigidBodyTree, RigidBodyFrame, FloatingBaseType, \
    AddModelInstanceFromUrdfStringSearchingInRosPackages, AddFlatTerrainToWorld
from pydrake.multibody.parsers import PackageMap

MODEL_PATH_ROOT = '/home/drl/PycharmProjects/underacuatedRoboics/model_world'
ground_urdf_path = os.path.join(MODEL_PATH_ROOT, 'world/plane.urdf')

class Laikago(object):
    def __init__(self, file_name , Is_fixed = False ):
        rb_tree = RigidBodyTree()
        world_frame = RigidBodyFrame("world_frame", rb_tree.world(), [0, 0, 0], [0, 0, 0])
        #AddFlatTerrainToWorld(rb_tree, 1000, 10)
        pmap = PackageMap()
        pmap.PopulateFromFolder(os.path.dirname(ground_urdf_path))
        AddModelInstanceFromUrdfStringSearchingInRosPackages(
            open(ground_urdf_path, 'r').read(),  # noqa
            pmap,
            os.path.dirname(ground_urdf_path),
            FloatingBaseType.kFixed,
            world_frame,
            rb_tree)
        robot_frame = RigidBodyFrame("robot_frame", rb_tree.world(), [0, 0, 1.0], [0, 0, 0])

        if Is_fixed :
            self.fixed_type = FloatingBaseType.kFixed
        else:
            self.fixed_type = FloatingBaseType.kRollPitchYaw
        # insert a robot from urdf files
        pmap = PackageMap()
        pmap.PopulateFromFolder(os.path.dirname(file_name))
        AddModelInstanceFromUrdfStringSearchingInRosPackages(
            open(file_name, 'r').read(),
            pmap,
            os.path.dirname(file_name),
            self.fixed_type,  # FloatingBaseType.kRollPitchYaw,
            robot_frame,
            rb_tree)



        self.rb_tree = rb_tree
        self.joint_limit_max = self.rb_tree.joint_limit_max
        self.joint_limit_min = self.rb_tree.joint_limit_min

        print(self.joint_limit_max)
        print(self.joint_limit_min)


motor_name = [
"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

