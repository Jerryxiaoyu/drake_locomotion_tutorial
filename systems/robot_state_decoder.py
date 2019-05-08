import numpy as np
import pydrake
import os


from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector, AbstractValue
from lcmt.robot_state_t import robot_state_t




class RobotStateDecoderLCM(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """

    def __init__(self,   mbp,  model_instance,  is_fixed = False):
        LeafSystem.__init__(self)
        self.mbp = mbp
        self.num_position = self.mbp.num_positions(model_instance)
        self.num_velocities = self.mbp.num_velocities(model_instance)

        self.num_states = self.num_position + self.num_velocities
        self.is_fixed = is_fixed

        self.num_controlled_q_ =self.mbp.num_actuators()
        self.n = 0
        self.num_output = 24


        # Declare lcm  input Port
        self.states_lcm_message_port_index = self.DeclareAbstractInputPort('lcm_message_port',
                                                                    AbstractValue.Make(robot_state_t)).get_index()
        # Declare output Port
        self.joint_state_results_port_index = self.DeclareVectorOutputPort('joint_state_results_port',
                                                                           BasicVector(self.num_controlled_q_ *2),
                                                                     self._OutputRobotState).get_index()

    def _OutputRobotState(self, context, output):

        message = self.EvalAbstractInput(context, self.states_lcm_message_port_index).get_value()

        timestamp = message.timestamp
        num_joints = message.num_joints
        joint_position = message.joint_position
        joint_velocity = message.joint_velocity
        print(message.timestamp)
        if len(joint_position) == 0:
            joint_position = np.zeros(12)
            joint_velocity = np.zeros(12)

        joint_state = np.concatenate((joint_position, joint_velocity), axis=0)

        output.SetFromVector(joint_state)

    def states_lcm_message_port(self):
        return self.get_input_port(self.states_lcm_message_port_index)

    def joint_state_results_port(self):
        return self.get_output_port(self.joint_state_results_port_index)
