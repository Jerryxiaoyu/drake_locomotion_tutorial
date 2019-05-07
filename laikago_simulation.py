import os, inspect

import numpy as np
import time
import pydrake
from pydrake.math import RigidTransform
from pydrake.systems.analysis import Simulator

from systems.RobotDiagram import LaikagoSimulationDiagram
from systems.laikago_config import *
from systems.utils import render_system_with_graphviz


is_graphviz = True
timestep = 0.001
sim_duration = 10.0
real_time_rate = 1

system_diagram = LaikagoSimulationDiagram(timestep,
                                          is_fixed=False, is_meshcat=True)

mbp = system_diagram.get_mbp()
diagram = system_diagram.get_diagram()

# generate system diagram using graphviz if not in test mode
if is_graphviz:
    render_system_with_graphviz(diagram, "systems/view.gv")

diagram_context = diagram.CreateDefaultContext()
mbp_context = diagram.GetMutableSubsystemContext(mbp, diagram_context)

# set initial pose for the robot.
if not system_diagram.is_fixed:
    X_WRobot = RigidTransform.Identity()
    X_WRobot.set_translation(system_diagram.laikago_initial_position_in_world_frame)
    mbp.SetFreeBodyPose(mbp_context, mbp.GetBodyByName("trunk", system_diagram.laikago_model),
                        X_WRobot)

# set initial posture of the robot.
for i, joint_angle in enumerate(ROBOT_STANCE_CONFIGURATION):
    laikago_joint = mbp.GetJointByName(JointName_list[i])
    laikago_joint.set_angle(context=mbp_context, angle=joint_angle)

# simulation
simulator = Simulator(diagram, diagram_context)  #
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(real_time_rate)
simulator.Initialize()

# need a few seconds to initiate the visualization
if system_diagram.is_meshcat:
    time.sleep(5)
print('Simulation is begining...')

lcm = system_diagram.lcm
# Run simulation
simulator.StepTo(sim_duration)
# lcm.StartReceiveThread()
# simulator.StepTo(sim_duration)
# lcm.StopReceiveThread()