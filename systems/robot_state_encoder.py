import numpy as np
import pydrake
import os


from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector, AbstractValue
from lcmt.robot_state_t import robot_state_t

class RobotStateEncoder(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block

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


class RobotStateEncoderLCM(LeafSystem):
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

        self.num_controlled_q_ =self.mbp.num_actuators()
        self.n = 0


        # Declare Input Port
        self.joint_state_results_port_index = self.DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                     self.num_states).get_index()

        # Declare lcm  Output Port
        self.lcm_message_port_index = self.DeclareAbstractOutputPort('lcm_message_port',
                                                                       alloc = self._Allocator,
                                                                       calc = self._OutputRobotState).get_index()

    def _Allocator(self):
        return AbstractValue.Make(robot_state_t)

    def _OutputRobotState(self, context, output):

        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()

        if self.is_fixed:
            q = states[:self.num_position]
            v = states[self.num_position:]

        else:
            q = states[7: self.num_position]  # [7:19]
            v = states[self.num_position + 6: self.num_states]  # 19+6 : 37

        message = robot_state_t()
        message.timestamp = context.get_time() * 1e3  # milliseconds

        message.num_joints = self.num_controlled_q_
        message.joint_position = q
        message.joint_velocity = v

        # print('t = {}  joint_pos = {}'.format(message.timestamp, message.joint_position))

        #print(message.timestamp)
        # print('t = {} '.format(message.timestamp ))
        output.set_value(message)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)

    def lcm_message_port(self):
        return self.get_output_port(self.lcm_message_port_index)

class JacoStateEncoder(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.

    Robot Plant --> this block

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
        if self.is_fixed:
            self.output_num = self.num_position *2
        else:
            self.output_num = (self.num_position - 7) *2
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
