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

apple_initial_position_in_world_frame = np.array([0, 0, 2.])
robot_initial_joint_angles = np.zeros(7)
robot_initial_joint_angles[0] = 0.2

timestep = 0.0002
sim_duration= 10.0
real_time_rate= 0.5
is_test=False

MODEL_PATH_ROOT = '/home/drl/PycharmProjects/underacuatedRoboics/model_world'
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

 
atlas_urdf_path = os.path.join(MODEL_PATH_ROOT,'atlas/atlas_minimal_contact_2.urdf')

# construct multibodyplant
mbp = MultibodyPlant(timestep)
scene_graph = SceneGraph()
mbp_parser = Parser(mbp, scene_graph)

# Add models

ground_model = mbp_parser.AddModelFromFile(
    file_name=ground_sdf_path, model_name='ground')
atlas_model = mbp_parser.AddModelFromFile(file_name=atlas_urdf_path, model_name='atlas_model')


# mount the plane to the ground
X_Wground = RigidTransform().Identity()
mbp.AddJoint(WeldJoint(name='weld_ground_to_world',
              parent_frame_P=mbp.world_body().body_frame(),
              child_frame_C= mbp.GetBodyByName("link", ground_model).body_frame(),
              X_PC=X_Wground))

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
# if not is_test:
#     viz = MeshcatVisualizer(scene_graph)
#     builder.AddSystem(viz)
#     builder.Connect(scene_graph.get_pose_bundle_output_port(),
#                     viz.get_input_port(0))

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



ConnectDrakeVisualizer(builder, scene_graph)
# Build diagram.
diagram = builder.Build()
# if not is_test:
#     viz.load()

# generate system diagram using graphviz if not in test mode
if not is_test:
    render_system_with_graphviz(diagram, "view.gv")

diagram_context = diagram.CreateDefaultContext()
mbp_context = diagram.GetMutableSubsystemContext(
    mbp, diagram_context)
# # Fix multibodyplant actuation input port to 0.

#
# # fix actuation input of atlas
mbp_context.FixInputPort(
    mbp.get_input_port(4).get_index(),
    np.zeros(mbp.get_input_port(4).size()))
print('DOF of the Atlas : {}'.format(mbp.get_input_port(4).size()))

# set initial pose for the atlas.
X_WAtlas = RigidTransform.Identity()
X_WAtlas.set_translation(apple_initial_position_in_world_frame)
mbp.SetFreeBodyPose(mbp_context, mbp.GetBodyByName("pelvis", atlas_model),
                           X_WAtlas)


# simulation
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(real_time_rate)
simulator.Initialize()

# Run simulation
simulator.StepTo(sim_duration)