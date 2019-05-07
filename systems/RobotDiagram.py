

import os
import numpy as np

import pydrake
from pydrake.common import AddResourceSearchPath, FindResourceOrThrow
from pydrake.multibody.tree import (UniformGravityFieldElement,
                                    WeldJoint, )
from pydrake.multibody.plant import MultibodyPlant, ConnectContactResultsToDrakeVisualizer
from pydrake.multibody.parsing import Parser
from pydrake.math import RigidTransform

from pydrake.systems.meshcat_visualizer import MeshcatVisualizer, MeshcatContactVisualizer
from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import SignalLogger

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from pydrake.systems.primitives import ConstantVectorSource, Sine
from pydrake.geometry import ConnectDrakeVisualizer

from pydrake.systems.controllers import InverseDynamicsController
from pydrake.attic.systems.controllers import RbtInverseDynamicsController
from pydrake.attic.multibody.rigid_body_tree import RigidBodyTree, RigidBodyFrame, FloatingBaseType, \
    AddModelInstanceFromUrdfStringSearchingInRosPackages
from pydrake.attic.multibody.parsers import PackageMap
from pydrake.lcm import DrakeLcm
from pydrake.geometry import DispatchLoadMessage

from laikago_config import *
from robot_state_encoder import RobotStateEncoder
from pd_ff_controller import PDAndFeedForwardController

class LaikagoSimulationDiagram(object):
    def __init__(self, timestep, is_fixed= False, is_meshcat = True):

        self.timestep = timestep
        self.logger_publish_period = 0.02
        self.is_fixed = is_fixed
        self.is_meshcat = is_meshcat
        self.enable_id_controller = False # only for actuated robot, that means it works when is_fixed = True
        self.robot_initial_joint_angles = ROBOT_STANCE_CONFIGURATION
        self.laikago_initial_position_in_world_frame = np.array([0, 0, 0.47])

        # these pid parameters are just for ID controller, not the pd controller
        self.kp_id = np.ones(12)*50
        self.ki_id = np.ones(12) * 0.
        self.kd_id = np.array([5, 5, 5, 5,
                               20, 20, 20, 20,
                               3, 3, 3, 3 ])

        self.laikago_rbt = self.Get_RigidBodyTree()
        # construct multibodyplant
        self.mbp = MultibodyPlant(self.timestep)
        self.scene_graph = SceneGraph()
        self.lcm = DrakeLcm()

        self._Construct_env()
        self.builder, self.SystemList, self.LoggerList = self._CreateBuilder()
        self.diagram = self.builder.Build()
        if self.is_meshcat:
            self.MeshcatVisualizerSystem.load()

        DispatchLoadMessage(self.scene_graph, self.lcm)


        # some parameters
        self.P_frame = self.get_robot_base_frame()
        self.W_frame = self.get_world_frame()

    def _Construct_env(self):
        ### Create an environment
        mbp_parser = Parser(self.mbp, self.scene_graph)
        # Add models
        self.laikago_model = mbp_parser.AddModelFromFile(file_name=laikago_urdf_path,
                                                         model_name='laikago')
        self.ground_model = mbp_parser.AddModelFromFile(file_name=ground_urdf_path,
                                                                model_name='ground')
        self.cinder_block_wide = mbp_parser.AddModelFromFile(file_name=cinder_block_wide_sdf_path,
                                                             model_name='block_angle_steps')

        # mount the plane to the ground
        X_Wground = RigidTransform().Identity()
        X_Wground.set_translation(np.array([0, 0, 0.]))
        self.mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
                           parent_frame_P=self.mbp.world_body().body_frame(),
                           child_frame_C=self.mbp.GetBodyByName("link", self.ground_model).body_frame(),
                           X_PC=X_Wground))

        # mount the block to the ground
        X_Wblock = RigidTransform().Identity()
        X_Wblock.set_translation(np.array([2, 2, 0.1]))
        self.mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
                           parent_frame_P=self.mbp.world_body().body_frame(),
                           child_frame_C=self.mbp.GetBodyByName("link", self.cinder_block_wide).body_frame(),
                           X_PC=X_Wblock))

        # mount the robot to the ground
        if self.is_fixed:
            X_Wrobot = RigidTransform().Identity()
            X_Wrobot.set_translation(np.array([0, 0, 0.8]))
            self.mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
                           parent_frame_P=self.mbp.world_body().body_frame(),
                           child_frame_C=self.mbp.GetBodyByName("trunk", self.laikago_model).body_frame(),
                           X_PC=X_Wrobot))

        # Add gravity
        self.mbp.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
        self.mbp.Finalize(self.scene_graph)
        assert self.mbp.geometry_source_is_registered()
        

    def _CreateBuilder(self):
        system_list = []
        logger_system_list = []

        builder = DiagramBuilder()

        MultibodyPlantSystem = builder.AddSystem(self.mbp)
        MultibodyPlantSystem.set_name('Multibody Plant')
        system_list.append(MultibodyPlantSystem)

        SceneGraphSystem = builder.AddSystem(self.scene_graph)
        SceneGraphSystem.set_name('Scene Graph')
        system_list.append(SceneGraphSystem)


        # Add meshcat visualizer if not in test mode
        if self.is_meshcat:
            self.MeshcatVisualizerSystem = builder.AddSystem(MeshcatVisualizer(self.scene_graph))
            builder.Connect(SceneGraphSystem.get_pose_bundle_output_port(),
                            self.MeshcatVisualizerSystem.get_input_port(0))
            system_list.append(SceneGraphSystem)
        else:
            ConnectDrakeVisualizer(builder, self.scene_graph, self.lcm)

        # Connect self.scene_graph to MBP for collision detection.
        builder.Connect(
            self.mbp.get_geometry_poses_output_port(),
            self.scene_graph.get_source_pose_port(self.mbp.get_source_id()))
        builder.Connect(
            self.scene_graph.get_query_output_port(),
            self.mbp.get_geometry_query_input_port())

        #ConnectContactResultsToDrakeVisualizer(builder, self.mbp, self.lcm)

        RobotContactVizSystem = builder.AddSystem(MeshcatContactVisualizer(self.MeshcatVisualizerSystem,
                                                                           force_threshold=1e-2,
                                                                           contact_force_scale= 100,
                                                                           plant = self.mbp))
        builder.Connect(self.scene_graph.get_pose_bundle_output_port(),
                        RobotContactVizSystem.get_input_port(0))
        builder.Connect(self.mbp.get_output_port(4),
                        RobotContactVizSystem.get_input_port(1))

        # Add system: Robot State Encoder
        RobotStateEncoderSystem = builder.AddSystem(
                                    RobotStateEncoder(self.mbp, self.laikago_model, is_fixed = self.is_fixed))  # force_sensor_info
        RobotStateEncoderSystem.set_name('Robot State Encoder')
        system_list.append(RobotStateEncoderSystem)
        builder.Connect(self.mbp.get_output_port(2),
                        RobotStateEncoderSystem.joint_state_results_input_port())





        # torque_vec = np.zeros(24)
        # torque_vec[0] = 1
        # Sine(amplitudes = torque_vec, frequencies = np.ones(24)* 1, phases = np.zeros(24))

        # desired states contains [q, v]
        desired_state = np.concatenate((self.robot_initial_joint_angles, np.zeros(12)))
        TrajectorySystem = builder.AddSystem(ConstantVectorSource(desired_state))
        system_list.append(TrajectorySystem)
        TrajectorySystem.set_name('Desired Trajectory System')

        # Add system : PD and feedforward controller
        PDAndFeedForwardControllerSystem = builder.AddSystem(
                                            PDAndFeedForwardController(self.laikago_rbt))
        PDAndFeedForwardControllerSystem.set_name('PD And Feedforward Controller')
        system_list.append(PDAndFeedForwardControllerSystem)

        builder.Connect(PDAndFeedForwardControllerSystem.motor_command_outport_port(),
                        self.mbp.get_input_port(3))
        builder.Connect(RobotStateEncoderSystem.joint_state_outport_port(),
                        PDAndFeedForwardControllerSystem.joint_state_results_input_port())
        builder.Connect(TrajectorySystem.get_output_port(0),
                        PDAndFeedForwardControllerSystem.joint_desired_state_input_port())

        if self.is_fixed:
            IDController = InverseDynamicsController(self.mbp,
                                                     self.kp_id, self.ki_id, self.kd_id,
                                                     has_reference_acceleration=False)
            IDControllerSystem = builder.AddSystem(IDController)
            IDControllerSystem.set_name('Inverse Dynamics Controller')
            system_list.append(IDControllerSystem)

            builder.Connect(RobotStateEncoderSystem.joint_state_outport_port(),
                            IDControllerSystem.get_input_port(0))

            # traj_src = builder.AddSystem(ConstantVectorSource(desired_state))
            builder.Connect(TrajectorySystem.get_output_port(0),
                            IDControllerSystem.get_input_port(1))

            if self.enable_id_controller:
                builder.Connect(IDControllerSystem.get_output_port(0),
                                PDAndFeedForwardControllerSystem.feedforward_input_port())
            else:
                ZerosSystem = builder.AddSystem(ConstantVectorSource(np.zeros(12)))
                ZerosSystem.set_name('Zeros Value System')
                system_list.append(ZerosSystem)
                builder.Connect(ZerosSystem.get_output_port(0),
                                PDAndFeedForwardControllerSystem.feedforward_input_port())
        else:
            ZerosSystem = builder.AddSystem(ConstantVectorSource(np.zeros(12)))
            ZerosSystem.set_name('Zeros Value System')
            system_list.append(ZerosSystem)
            builder.Connect(ZerosSystem.get_output_port(0),
                            PDAndFeedForwardControllerSystem.feedforward_input_port())
        ##########################################################################################
        ##----------------------Add Logger System to the plant---------------------------------##

        # Add robot state logger
        RobotStateLog = builder.AddSystem(SignalLogger(self.mbp.get_state_output_port(self.laikago_model).size()))
        RobotStateLog.set_name('Robot State Logger')
        RobotStateLog.DeclarePeriodicPublish(self.logger_publish_period)
        builder.Connect(self.mbp.get_state_output_port(self.laikago_model),
                        RobotStateLog.get_input_port(0))
        logger_system_list.append(RobotStateLog)

        if self.is_fixed:
            # Add logger
            FF_CommandLog = builder.AddSystem(SignalLogger(IDControllerSystem.get_output_port(0).size()))
            FF_CommandLog.set_name('FeedForward Command Logger')
            FF_CommandLog.DeclarePeriodicPublish(self.logger_publish_period)
            builder.Connect(IDControllerSystem.get_output_port(0), FF_CommandLog.get_input_port(0))
            logger_system_list.append(FF_CommandLog)

        # Add command logger
        CommandLog = builder.AddSystem(SignalLogger(PDAndFeedForwardControllerSystem.motor_command_outport_port().size()))
        CommandLog.set_name('Command Logger')
        CommandLog.DeclarePeriodicPublish(self.logger_publish_period)
        builder.Connect(PDAndFeedForwardControllerSystem.motor_command_outport_port(), CommandLog.get_input_port(0))
        logger_system_list.append(CommandLog)

        return builder, system_list, logger_system_list

    def get_mbp(self):
        return self.mbp

    def get_diagram(self):
        return self.diagram

    def Get_RigidBodyTree(self):
        '''
        get a rigidbodytree that describes the laikago without any instance.
        :return: rigidbodytree of laiakgo
        '''
        rbt = RigidBodyTree()
        pmap = PackageMap()
        pmap.PopulateFromFolder(MODEL_PATH_ROOT)
        laikago_frame = RigidBodyFrame("laikago_frame", rbt.world(),
                                       [0, 0, 0], [0, 0, 0])
        AddModelInstanceFromUrdfStringSearchingInRosPackages(
            open(laikago_urdf_path, 'r').read(),
            pmap,
            os.path.join(MODEL_PATH_ROOT, 'laikago_v1'),
            FloatingBaseType.kFixed,
            laikago_frame,
            rbt)

        return rbt


    def get_robot_base_frame(self):
        return self.mbp.GetFrameByName('trunk', self.laikago_model)
    def get_world_frame(self):
        return self.mbp.world_body().body_frame()



    def get_COM_in_worldframe(self, context):
        '''
        :param context:
        :return: np.array([3,]) The center of mass of the robot is expressed in the world frame.
        '''

        q = needToknow
        kineCache = self.laikago_rbt.doKinematics(q)


        Bcom_P = RigidTransform().Identity()
        Bcom_P.set_translation(self.laikago_rbt.centerOfMass(kineCache))

        X_WP = self.mbp.CalcRelativeTransform(context, frame_A=self.W_frame, frame_B=self.get_robot_base_frame())

        Bcom_transl_W = X_WP.multiply(Bcom_P).translation()

        return Bcom_transl_W
