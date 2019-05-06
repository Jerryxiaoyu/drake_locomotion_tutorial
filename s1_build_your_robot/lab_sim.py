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




def render_system_with_graphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)


table_top_z_in_world = 0.736 + 0.057 / 2

apple_initial_position_in_world_frame = np.array([-0.2, -0.2, 10.])
robot_initial_joint_angles = np.zeros(7)
robot_initial_joint_angles[0] = 0.2

timestep = 0.0002
sim_duration= 10.0
real_time_rate= 0.5
is_test=False


# set the path of models in the env
iiwa_sdf_path = os.path.join(
    pydrake.getDrakePath(),
    "manipulation", "models", "iiwa_description", "sdf",
    "iiwa14_no_collision.sdf")

wsg50_sdf_path = os.path.join(
    pydrake.getDrakePath(),
    "manipulation", "models", "wsg_50_description", "sdf",
    "schunk_wsg_50.sdf")

table_sdf_path = os.path.join(
    pydrake.getDrakePath(),
    "examples", "kuka_iiwa_arm", "models", "table",
    "extra_heavy_duty_table_surface_only_collision.sdf")

apple_sdf_path = os.path.join(pydrake.getDrakePath(),
  "examples", "kuka_iiwa_arm", "models", "objects", "yellow_post.urdf")

ground_sdf_path = os.path.join(pydrake.getDrakePath(),
   "examples/atlas/sdf/ground_plane/model.sdf")
block_angle_steps_1 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_angle_steps_1/model.sdf")
block_angle_steps_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_angle_steps_2/model.sdf")
block_level_steps_1 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_level_steps_1/model.sdf")
block_level_steps_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/block_level_steps_2/model.sdf")
cinder_block_2 =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/cinder_block_2/model.sdf")
cinder_block_wide =os.path.join(pydrake.getDrakePath(), "examples/atlas/sdf/cinder_block_wide/model.sdf")

bot_lab_path = os.path.join("/home/drl/PycharmProjects/underacuatedRoboics", 'model_world/botlab/botlab.sdf')


# construct multibodyplant
mbp = MultibodyPlant(timestep)
scene_graph = SceneGraph()
mbp_parser = Parser(mbp, scene_graph)

# Add models
# iiwa_model = mbp_parser.AddModelFromFile(
#     file_name=iiwa_sdf_path, model_name='robot')
# gripper_model = mbp_parser.AddModelFromFile(
#     file_name=wsg50_sdf_path, model_name='gripper')
# table_model = mbp_parser.AddModelFromFile(
#     file_name=table_sdf_path, model_name='table')
# table2_model = mbp_parser.AddModelFromFile(
#     file_name=table_sdf_path, model_name='table2')
# apple_model = mbp_parser.AddModelFromFile(
#     file_name=apple_sdf_path, model_name='apple')
# ground_model = mbp_parser.AddModelFromFile(
#     file_name=ground_sdf_path, model_name='ground'
# )
bot_lab_model = mbp_parser.AddAllModelsFromFile(file_name=bot_lab_path)

# # mount the gripper to iiwa
# X_EeGripper = Isometry3.Identity()
# X_EeGripper.set_translation([0, 0, 0.081])
# X_EeGripper.set_rotation(RollPitchYaw(np.pi / 2, 0, np.pi / 2).ToRotationMatrix().matrix())
#
# mbp.AddJoint(
#     WeldJoint(name="weld_gripper_to_robot_ee",
#               parent_frame_P=mbp.GetBodyByName(
#                   "iiwa_link_7", iiwa_model).body_frame(),
#               child_frame_C=mbp.GetBodyByName(
#                   "body", gripper_model).body_frame(),
#               X_PC=X_EeGripper))
#
# # mount the table to the ground
# mbp.AddJoint(
#     WeldJoint(name="weld_table_to_world",
#               parent_frame_P=mbp.world_body().body_frame(),
#               child_frame_C=mbp.GetBodyByName(
#                   "link", table_model).body_frame(),
#               X_PC=Isometry3.Identity()))
#
# # mount the iiwa to the table
# X_WRobot = Isometry3.Identity()
# X_WRobot.set_translation([0, 0, table_top_z_in_world])
# mbp.AddJoint(
#     WeldJoint(name="weld_robot_to_world",
#               parent_frame_P=mbp.world_body().body_frame(),
#               child_frame_C=mbp.GetBodyByName(
#                   "iiwa_link_0", iiwa_model).body_frame(),
#               X_PC=X_WRobot))
#
# # mount the table2 to the ground
# X_WTable2 = Isometry3.Identity()
# X_WTable2.set_translation([0.8, 0, 0])
# mbp.AddJoint(
#     WeldJoint(name='weld_table2_to_world',
#               parent_frame_P=mbp.world_body().body_frame(),
#               child_frame_C=mbp.GetBodyByName("link", table2_model).body_frame(),
#               X_PC=X_WTable2))
#
# mount the table2 to the ground
# X_Wground = Isometry3.Identity()
#
# mbp.AddJoint(
#     WeldJoint(name='weld_ground_to_world',
#               parent_frame_P=mbp.world_body().body_frame(),
#               child_frame_C= mbp.GetBodyByName("link", ground_model).body_frame(),
#               X_PC=X_Wground))


#mount the lab to the ground
X_Wlab = RigidTransform().Identity()
X_Wlab.set_rotation(RollPitchYaw(np.pi / 2, 0, 0).ToRotationMatrix())

for i,obj in enumerate(bot_lab_model):
    mbp.AddJoint(
        WeldJoint(name="weld_lab_to_world"+str(i),
                  parent_frame_P=mbp.world_body().body_frame(),
                  child_frame_C=mbp.get_body(mbp.GetBodyIndices(obj)[0]).body_frame(),
                  X_PC=X_Wlab))



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
if not is_test:
    viz = MeshcatVisualizer(scene_graph)
    builder.AddSystem(viz)
    builder.Connect(scene_graph.get_pose_bundle_output_port(),
                    viz.get_input_port(0))

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

# Build diagram.
diagram = builder.Build()
if not is_test:
    viz.load()

# generate system diagram using graphviz if not in test mode
if not is_test:
    render_system_with_graphviz(diagram, "view.gv")

diagram_context = diagram.CreateDefaultContext()
mbp_context = diagram.GetMutableSubsystemContext(
    mbp, diagram_context)
# # Fix multibodyplant actuation input port to 0.

#
# # fix actuation input of iiwa
# mbp_context.FixInputPort(
#     mbp.get_input_port(3).get_index(),
#     np.zeros(mbp.get_input_port(3).size()))
# print('DOF of the iiwa : {}'.format(mbp.get_input_port(3).size()))
#
# # fix actuation input of gripper
# mbp_context.FixInputPort(
#     mbp.get_input_port(4).get_index(),
#     np.zeros(mbp.get_input_port(4).size()))
# print('DOF of the griper : {}'.format(mbp.get_input_port(4).size()))
#
# # set initial pose for the apple.
# X_WApple = Isometry3.Identity()
# X_WApple.set_translation(apple_initial_position_in_world_frame)
# mbp.SetFreeBodyPose(mbp_context, mbp.GetBodyByName("base", apple_model),
#                            X_WApple)
#
#
# # set initial posture of the iiwa arm.
# for i, joint_angle in enumerate(robot_initial_joint_angles):
#     iiwa_joint = mbp.GetJointByName("iiwa_joint_%d"%(i+1))
#     iiwa_joint.set_angle(context=mbp_context, angle=joint_angle)


# simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(real_time_rate)
simulator.Initialize()

# Run simulation
simulator.StepTo(sim_duration)