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
    AddModelInstanceFromUrdfStringSearchingInRosPackages
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector

from pydrake.attic.multibody.rigid_body_plant import RigidBodyPlant, DrakeVisualizer
from pydrake.lcm import DrakeLcm


from laikago import Laikago

motor_name = [
"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]

def render_system_with_graphviz(system, output_file="system_view.gz"):
    ''' Renders the Drake system (presumably a diagram,
    otherwise this graph will be fairly trivial) using
    graphviz to a specified file. '''
    from graphviz import Source
    string = system.GetGraphvizString()
    src = Source(string)
    src.render(output_file, view=False)

class RobotStateEncoder(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """
    def __init__(self):
        LeafSystem.__init__(self )
        # self.rb_tree = rb_tree
        # self.num_position = self.rb_tree.get_num_positions()
        # self.num_controlled_q_ = self.rb_tree.get_num_actuators()

        self.input_num = 36
        self.output_num = 24
        self.n = 0

        # Input Port
        self.joint_state_results_port_index = self._DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.input_num).get_index()
        #         self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                        AbstractValue.Make(ContactResults)).get_index()
        # Output Port

        self.joint_state_outport_index = self._DeclareVectorOutputPort('state_output_port', BasicVector(self.output_num ),
                                                                       self._OutputRobotState).get_index()


    def _OutputRobotState(self, context, output):


        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()

        q = states[6:18]
        u = states[18+6:36]
        # q= states[:12]
        # u= states[12:]

        # get joint state from full state
        joint_state = np.concatenate(( q,
                                      u), axis=0)

        #print(joint_state)

        # self.n = self.n +1
        # if self.n % 100 ==0:
        #     print self.n

        output.SetFromVector(joint_state)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)
    def joint_state_outport_port(self):
        return self.get_output_port(self.joint_state_outport_index)


class PDAndFeedForwardController(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """
    def __init__(self, rb_tree, desired_state, Kp, Kd):
        LeafSystem.__init__(self)
        self.rb_tree = rb_tree
        self.num_position = self.rb_tree.get_num_positions()
        self.num_controlled_q_ = self.rb_tree.get_num_actuators()

        self.desired_state = desired_state
        self.input_num = 24
        self.output_num = 12
        self.n = 0

        self.Kp = Kp
        self.Kd = Kd

        # Input Port
        self.joint_state_results_port_index = self._DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.input_num).get_index()
        #         self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                        AbstractValue.Make(ContactResults)).get_index()
        # Output Port

        self.motor_command_outport_index = self._DeclareVectorOutputPort('motor_command_output_port', BasicVector(self.output_num ),
                                                                       self._OutputControlCommand).get_index()


    def _OutputControlCommand(self, context, output):


        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()


        q= states[:12]
        v= states[12:]

        q_d = self.desired_state[:12]
        v_d = self.desired_state[12:]

        #print(context.get_time())
        #print('q = {}, q_d ={}'.format(q, q_d))
        #print('v = {}, v_d ={}'.format(v, v_d))
        command = self.Kp * (q_d - q) + self.Kd * (v_d - v)
        #command = np.zeros(12)
        #command[4] = np.sin(30*context.get_time()) *30

        #command = p.MakeActuationMatrix().dot(command)
        #print('command ={}'.format(command))

        output.SetFromVector(command)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)
    def motor_command_outport_port(self):
        return self.get_output_port(self.motor_command_outport_index)

MODEL_PATH_ROOT = '/home/drl/PycharmProjects/underacuatedRoboics/model_world'
laikago_urdf_path = os.path.join(MODEL_PATH_ROOT, 'laikago_v1/laikago.urdf')
ground_urdf_path = os.path.join(MODEL_PATH_ROOT, 'world/plane.urdf')

timestep = 0.002
sim_duration= 10.0
real_time_rate= 1


laikago = Laikago(laikago_urdf_path, Is_fixed= False)
rb_tree = laikago.rb_tree


num_pos = rb_tree.get_num_positions()
num_actuators = rb_tree.get_num_actuators()

zero_config = np.zeros((rb_tree.get_num_actuators() * 2, 1))

lcm =   DrakeLcm()

# Drake diagram
builder = DiagramBuilder()



plant = builder.AddSystem(RigidBodyPlant(rb_tree, timestep))
plant.set_name('plant')

# # Visualizer
drake_visualizer = DrakeVisualizer(rb_tree, lcm, enable_playback= False)
visualizer_publisher = builder.AddSystem(drake_visualizer)
visualizer_publisher.set_name('visualizer_publisher')
visualizer_publisher.set_publish_period(1.0/60.0)


builder.Connect(plant.state_output_port(),
                visualizer_publisher.get_input_port(0))



# # # Robot State Encoder
# robot_state_encoder = builder.AddSystem(RobotStateEncoder())  # force_sensor_info
# robot_state_encoder.set_name('robot_state_encoder')
#
# builder.Connect(plant.state_output_port(),
#                 robot_state_encoder.joint_state_results_input_port())

print(rb_tree.get_num_positions())
#print(plant.GetJointByName(motor_name[0]).velocity_upper_limits())

kp = np.ones(12) * 20
ki = np.ones(12) * 0.0
kd = np.ones(12) * 0.5


init_state = np.zeros(12)
init_state[1] = 0
#init_state = np.array([0,0.62,-1.26,0,0.62,-1.26,0,0.62,-1.26,0,0.62,-1.26,])
desired_state = np.concatenate((init_state, np.zeros(12)))


# robot controller
# pd_controller = PDAndFeedForwardController(rb_tree, desired_state, kp, kd)
# pd_controller_system = builder.AddSystem(pd_controller)
#
# builder.Connect(pd_controller_system.motor_command_outport_port(),
#                 plant.get_input_port(0))
#
# builder.Connect(robot_state_encoder.joint_state_outport_port(),
#                 pd_controller_system.joint_state_results_input_port())


print(plant.get_input_port(0).size())
torque = 0.0
torque_system = builder.AddSystem(ConstantVectorSource(
                                np.ones((plant.get_input_port(0).size(), 1))*torque))
builder.Connect(torque_system.get_output_port(0),
                plant.model_instance_actuator_command_input_port(1))

# Build diagram.
diagram = builder.Build()

render_system_with_graphviz(diagram, "view.gv")

num_pos = rb_tree.get_num_positions()
init_state =  np.concatenate((np.zeros(num_pos), np.zeros(num_pos)), axis=0)
# simulation setting
diagram_context = diagram.CreateDefaultContext()


plant_context = diagram.GetMutableSubsystemContext(plant,diagram_context)
plant.set_state_vector(plant_context, init_state)

# laikago_initial_position_in_world_frame = np.array([0, 0 , 0.7])
# X_WRobot = RigidTransform.Identity()
# X_WRobot.set_translation(laikago_initial_position_in_world_frame)
# plant.SetFreeBodyPose(plant_context, plant.GetBodyByName("base", laikago_model),
#                    X_WRobot)


# robot_initial_joint_angles = np.zeros(12)
#
# for i, joint_angle in enumerate(robot_initial_joint_angles):
#     laikago_joint = plant.GetJointByName(motor_name[i])
#     laikago_joint.set_angle(context=plant_context, angle=joint_angle)


simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(False)
simulator.set_target_realtime_rate(1)
simulator.Initialize()
simulator.get_mutable_integrator().set_target_accuracy(1e-3)




# t0 = time.time()
lcm.StartReceiveThread()
simulator.StepTo(sim_duration)
lcm.StopReceiveThread()
# tf = time.time()