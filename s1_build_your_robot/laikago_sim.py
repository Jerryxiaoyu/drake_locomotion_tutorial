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
from pydrake.common.eigen_geometry import Isometry3
from pydrake.math import RigidTransform

from pydrake.math import RollPitchYaw

from pydrake.systems.meshcat_visualizer import MeshcatVisualizer
from pydrake.geometry import SceneGraph
from pydrake.systems.primitives import SignalLogger

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from pydrake.systems.primitives import ConstantVectorSource
from pydrake.geometry import ConnectDrakeVisualizer

def render_system_with_graphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)


table_top_z_in_world = 0.736 + 0.057 / 2

laikago_initial_position_in_world_frame = np.array([0, 0, 0.6])
robot_initial_joint_angles = np.zeros(12)
robot_initial_joint_angles[0] = 0.2

timestep = 0.002
sim_duration= 10.0
real_time_rate= 0.5
is_test=False


motor_name = [
"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

is_meshcat = False

MODEL_PATH_ROOT = '/home/drl/PycharmProjects/underacuatedRoboics/model_world'

# set the path of models in the env

block_angle_steps_1 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_angle_steps_1/model.sdf")
block_angle_steps_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_angle_steps_2/model.sdf")
block_level_steps_1 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_level_steps_1/model.sdf")
block_level_steps_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_level_steps_2/model.sdf")
cinder_block_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/cinder_block_2/model.sdf")
cinder_block_wide =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/cinder_block_wide/model.sdf")


laikago_urdf_path = os.path.join(MODEL_PATH_ROOT, 'laikago_v1/laikago2.urdf')
ground_urdf_path = os.path.join(MODEL_PATH_ROOT, 'world/plane.urdf')

# construct multibodyplant
mbp = MultibodyPlant(timestep)
scene_graph = SceneGraph()
mbp_parser = Parser(mbp, scene_graph)

### Create an environment
# Add models
ground_model = mbp_parser.AddModelFromFile(file_name=ground_urdf_path, model_name='ground')
laikago_model = mbp_parser.AddModelFromFile(file_name=laikago_urdf_path, model_name='laikago')

block_angle_steps_1 = mbp_parser.AddModelFromFile(file_name=cinder_block_wide, model_name='block_angle_steps')

# mount the plane to the ground
X_Wground = RigidTransform().Identity()
X_Wground.set_translation(np.array([0, 0, 0.]))
mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
              parent_frame_P=mbp.world_body().body_frame(),
              child_frame_C= mbp.GetBodyByName("link", ground_model).body_frame(),
              X_PC=X_Wground))

# mount the block to the ground
X_Wblock = RigidTransform().Identity()
X_Wblock.set_translation(np.array([2, 2, 0.1]))
mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
              parent_frame_P=mbp.world_body().body_frame(),
              child_frame_C= mbp.GetBodyByName("link", block_angle_steps_1).body_frame(),
              X_PC=X_Wblock))


# Add gravity
mbp.AddForceElement(UniformGravityFieldElement([0, 0, -9.81]))
mbp.Finalize(scene_graph)
assert mbp.geometry_source_is_registered()

#%% Build drake diagram
# Drake diagram
builder = DiagramBuilder()
builder.AddSystem(scene_graph)
builder.AddSystem(mbp)

# Add meshcat visualizer if not in test mode
viz = None
if is_meshcat:
    viz = MeshcatVisualizer(scene_graph)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))
else:
    ConnectDrakeVisualizer(builder, scene_graph)

# Connect scene_graph to MBP for collision detection.
builder.Connect(
    mbp.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(mbp.get_source_id()))
builder.Connect(
    scene_graph.get_query_output_port(),
    mbp.get_geometry_query_input_port())


# Add logger
state_log = builder.AddSystem(SignalLogger(mbp.get_continuous_state_output_port().size()))
state_log.DeclarePeriodicPublish(0.02)
builder.Connect(mbp.get_continuous_state_output_port(), state_log.get_input_port(0))



print(mbp.get_input_port(4).size())
torque = 1.0
torque_system = builder.AddSystem(ConstantVectorSource(
                                np.ones((mbp.get_input_port(4).size(), 1))*torque))
builder.Connect(torque_system.get_output_port(0),
                mbp.get_input_port(4))



# Build diagram.
diagram = builder.Build()
if is_meshcat:
    viz.load()

# generate system diagram using graphviz if not in test mode
if not is_test:
    render_system_with_graphviz(diagram, "view.gv")

diagram_context = diagram.CreateDefaultContext()
mbp_context = diagram.GetMutableSubsystemContext(
    mbp, diagram_context)


 
# set initial pose for the robot.
X_WRobot = RigidTransform.Identity()
X_WRobot.set_translation(laikago_initial_position_in_world_frame)
mbp.SetFreeBodyPose(mbp_context, mbp.GetBodyByName("base", laikago_model),
                    X_WRobot)

# set initial posture of the iiwa arm.
for i, joint_angle in enumerate(robot_initial_joint_angles):
    laikago_joint = mbp.GetJointByName(motor_name[i])
    laikago_joint.set_angle(context=mbp_context, angle=joint_angle)

# simulation
simulator = Simulator(diagram, diagram_context) #
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(real_time_rate)
simulator.Initialize()

# Run simulation
simulator.StepTo(sim_duration)