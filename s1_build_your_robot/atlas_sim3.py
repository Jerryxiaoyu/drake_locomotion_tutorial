"""Provides an example translation of `cart_pole_passive_simluation.cc`."""
import os
import argparse

import numpy as np

import pydrake
from pydrake.common import FindResourceOrThrow
from pydrake.geometry import (ConnectDrakeVisualizer, SceneGraph)
from pydrake.lcm import DrakeLcm
from pydrake.multibody.multibody_tree import UniformGravityFieldElement
from pydrake.multibody.multibody_tree.multibody_plant import MultibodyPlant
#from pydrake.multibody.multibody_tree.parsing import AddModelFromSdfFile
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.attic.multibody.rigid_body_tree import (RigidBodyTree,
                                               RigidBodyFrame,
                                               AddModelInstanceFromUrdfStringSearchingInRosPackages,
                                               FloatingBaseType,
                                               )
from pydrake.attic.multibody.rigid_body_plant import RigidBodyPlant, DrakeVisualizer
from meshcat_rigid_body_visualizer import MeshcatRigidBodyVisualizer
from pydrake.systems.primitives import SignalLogger
from pydrake.systems.primitives import ConstantVectorSource


def setupAtlasExample():
    rbt = RigidBodyTree()
    world_frame = RigidBodyFrame("world_frame", rbt.world(),
                                 [0, 0, 0], [0, 0, 0])
    from pydrake.multibody.parsers import PackageMap
    pmap = PackageMap()
    pmap.PopulateFromFolder(os.path.join(pydrake.getDrakePath(), "examples"))
    print(pmap)
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open('plane.urdf', 'r').read(),  # noqa
        pmap,
        pydrake.getDrakePath() + "/examples/",
        FloatingBaseType.kFixed,
        world_frame,
        rbt)

    # AddFlatTerrainToWorld(rbt, 1000, 1)

    Atlas_frame = RigidBodyFrame("Atlas_frame", rbt.world(),
                                 [0, 0, 1.5], [0, 0, 0])
    AddModelInstanceFromUrdfStringSearchingInRosPackages(
        open(pydrake.getDrakePath() + "/examples/atlas/urdf/atlas_convex_hull.urdf", 'r').read(),
        # noqa  atlas_minimal_contact.urdf
        pmap,
        pydrake.getDrakePath() + "/examples/",
        FloatingBaseType.kRollPitchYaw,
        Atlas_frame,
        rbt)

    Tview = np.array([[1., 0., 0., 0.],
                      [0., 0., 1., 0.],
                      [0., 0., 0., 1.]],
                     dtype=np.float64)
    pbrv = MeshcatRigidBodyVisualizer(rbt)

    return rbt, pbrv



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--simulation_time", type=float, default=10.0,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--time_step", type=float, default=0.005,
        help="If greater than zero, the plant is modeled as a system with "
             "discrete updates and period equal to this time_step. "
             "If 0, the plant is modeled as a continuous system.")
    args = parser.parse_args()




    builder = DiagramBuilder()
    # Create a block diagram

    # 1. set up rigid body model
    rbt, pbrv = setupAtlasExample()
    robot_plant = RigidBodyPlant(rbt, timestep=args.time_step)

    rbplant_sys = builder.AddSystem(robot_plant)

    nx = rbt.get_num_positions() + rbt.get_num_velocities()
    # Visualize
    visualizer = builder.AddSystem(pbrv)
    builder.Connect(rbplant_sys.get_output_port(0),
                    visualizer.get_input_port(0))

    # --control system
    torque = 1.0
    torque_system = builder.AddSystem(ConstantVectorSource(
        np.ones((rbt.get_num_actuators(), 1)) * torque))
    builder.Connect(torque_system.get_output_port(0),
                    rbplant_sys.get_input_port(0))


    # And also log
    signalLogRate = 60
    signalLogger = builder.AddSystem(SignalLogger(nx))
    signalLogger._DeclarePeriodicPublish(1. / signalLogRate, 0.0)
    builder.Connect(rbplant_sys.get_output_port(0),
                    signalLogger.get_input_port(0))

    # simulation settings
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # TODO(russt): Clean up state vector access below.
    state = simulator.get_mutable_context().get_mutable_state() \
        .get_mutable_continuous_state().get_mutable_vector()

    duration =10
    simulator.StepTo(duration)
if __name__ == "__main__":
    main()