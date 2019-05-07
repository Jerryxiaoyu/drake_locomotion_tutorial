import numpy as np
import pydrake
import os


from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector

from laikago_config import *
Kp = np.array([
            50.0, 50.0, 50.0, 50.0,
            50.0, 50.0, 50.0, 50.0,
            50.0, 50.0, 50.0, 50.0,
        ])
# kp = np.ones(12) * 50
Ki = np.ones(12) * 0.
Kd = np.array([
    5, 5, 5, 5,
    20, 20, 20, 20,
    3, 3, 3, 3
])


class PDAndFeedForwardController(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """

    def __init__(self, rb_tree, Kp = Kp, Kd = Kd):
        LeafSystem.__init__(self)
        self.rb_tree = rb_tree

        self.num_controlled_q = self.rb_tree.get_num_actuators()

        # self.desired_position = desired_position
        self.num_states = self.num_controlled_q *2
        self.num_output = self.num_controlled_q
        self.feedforward_num = self.num_controlled_q
        self.n = 0

        self.Kp = Kp
        self.Kd = Kd

        # Input Port
        self.joint_state_results_port_index = self.DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                    self.num_states).get_index()
        #         self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                        AbstractValue.Make(ContactResults)).get_index()
        self.joint_desired_state_port_index = self.DeclareInputPort('joint_desired_state_port',
                                                                     PortDataType.kVectorValued,
                                                                    self.num_states).get_index()

        self.feedforward_input_port_index = self.DeclareInputPort('feedforward_input_port',
                                                                   PortDataType.kVectorValued,
                                                                   self.feedforward_num).get_index()
        # Output Port  motor torque
        self.motor_command_outport_index = self.DeclareVectorOutputPort('motor_command_output_port',
                                                                         BasicVector(self.num_output),
                                                                         self._OutputControlCommand).get_index()

    def _OutputControlCommand(self, context, output):
        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()
        desired_states = self.EvalVectorInput(context, self.joint_desired_state_port_index).get_value()
        feedforward = self.EvalVectorInput(context, self.feedforward_input_port_index).get_value()

        q = states[:self.num_controlled_q]
        v = states[self.num_controlled_q:]

        q_d = desired_states[:self.num_controlled_q]
        v_d = desired_states[self.num_controlled_q:]

        #print(context.get_time())
        #print('q = {}, q_d ={}'.format(q, q_d))
        # print('e= ', q_d - q)
        # print('v = {}, v_d ={}'.format(v, v_d))
        command = self.Kp * (q_d - q) + self.Kd * (v_d - v) + feedforward

        command = command.reshape((-1, 1))

        # TODO I found that MakeActuationMatrix() turns out the transpose of the expected one.
        # command = mbp.MakeActuationMatrix().dot(command)
        # print('command ={}'.format(command))

        command = np.matmul(ACTUATOR_CHOICE_MATRIX, command)

        # TODO modify clipping torque function
        command = np.clip(command, -MAX_TORQUE, MAX_TORQUE)
        output.SetFromVector(command)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)

    def joint_desired_state_input_port(self):
        return self.get_input_port(self.joint_desired_state_port_index)

    def motor_command_outport_port(self):
        return self.get_output_port(self.motor_command_outport_index)

    def feedforward_input_port(self):
        return self.get_input_port(self.feedforward_input_port_index)

