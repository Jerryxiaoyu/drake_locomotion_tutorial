import numpy as np
import pydrake
import os


from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector

class RobotStateEncoder(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block --> lcm Publisher

    Input Port :
        -- joint_state_results_port : kVectorValued  , [n *2 , 1]  the positions and velocties of joints

    Output Port:
        -- lcm_message_port : AbstractValue, lcm publisher type

    """

    def __init__(self, mbp,  model_instance,  is_fixed = False):
        LeafSystem.__init__(self)
        self.mbp = mbp
        self.num_position = self.mbp.num_positions(model_instance)
        self.num_velocities = self.mbp.num_velocities(model_instance)

        self.num_states = self.num_position + self.num_velocities
        self.is_fixed = is_fixed

        self.output_num = 24
        self.n = 0

        # Declare Input Port
        self.joint_state_results_port_index = self.DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.num_states).get_index()

        # Declare Output Port
        self.joint_state_outport_index = self.DeclareVectorOutputPort('state_output_port',
                                                                       BasicVector(self.output_num),
                                                                       self._OutputRobotState).get_index()

    def _OutputRobotState(self, context, output):

        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()

        if self.is_fixed:
            q = states[:self.num_position]
            v = states[self.num_position:]

        else:
            q = states[7: self.num_position]  # [7:19]
            v = states[self.num_position + 6: self.num_states]  # 19+6 : 37

        # get joint state from full state
        joint_state = np.concatenate((q, v), axis=0)

        # print(joint_state)

        # self.n = self.n +1
        # if self.n % 100 ==0:
        #     print self.n

        output.SetFromVector(joint_state)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)

    def joint_state_outport_port(self):
        return self.get_output_port(self.joint_state_outport_index)
