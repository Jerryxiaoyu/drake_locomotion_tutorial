import numpy as np
import pydrake
import os


from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector, AbstractValue

from pydrake.math import RigidTransform

class Claculator(LeafSystem):
    """
    A block system that outputs all state infomation to a lcm publisher.


    """

    def __init__(self, system_diagram):
        LeafSystem.__init__(self)
        self.system_diagram = system_diagram
        self.mbp = self.system_diagram.mbp
        self.rb_tree = self.system_diagram.Get_RigidBodyTree()

        self.P_frame = self.system_diagram.get_robot_base_frame()
        self.W_frame = self.system_diagram.get_world_frame()

        self.num_controlled_q = self.rb_tree.get_num_actuators()

        # self.desired_position = desired_position
        self.num_states = self.num_controlled_q *2
        self.num_output = self.num_controlled_q

        self.n = 0


        # Input Port
        self.joint_state_results_port_index = self.DeclareInputPort('joint_state_results_port',
                                                                     PortDataType.kVectorValued,
                                                                    self.num_states).get_index()


        # self.kinematics_results_port_index = self.DeclareAbstractInputPort('kinematics_results_port',
        #                                                                    AbstractValue.Make(ContactResults)).get_index()
        #
        # self.contact_results_port_index = self._DeclareAbstractInputPort('contact_results_port',
        #                                                                   AbstractValue.Make(ContactResults)).get_index()

        # self.joint_desired_state_port_index = self.DeclareInputPort('joint_desired_state_port',
        #                                                              PortDataType.kVectorValued,
        #                                                             self.num_states).get_index()


        # Output Port  motor torque
        self.motor_command_outport_index = self.DeclareVectorOutputPort('motor_command_output_port',
                                                                         BasicVector(self.num_output),
                                                                         self._OutputControlCommand).get_index()

    def _OutputControlCommand(self, context, output):
        states = self.EvalVectorInput(context, self.joint_state_results_port_index).get_value()

        q = states[:self.num_controlled_q]
        v = states[self.num_controlled_q:]

        kineCache = self.laikago_rbt.doKinematics(q)
        Bcom_P = RigidTransform().Identity()
        Bcom_P.set_translation(self.laikago_rbt.centerOfMass(kineCache))
        X_WP = self.mbp.CalcRelativeTransform(context, frame_A=self.W_frame, frame_B=self.get_robot_base_frame())

        Bcom_transl_W = X_WP.multiply(Bcom_P).translation()

        if self.n % 100==0:
            print('t = ',context.get_time(), " : ", Bcom_transl_W)
        self.n +=1

        command = np.ones(12)
        output.SetFromVector(command)

    def joint_state_results_input_port(self):
        return self.get_input_port(self.joint_state_results_port_index)


    def motor_command_outport_port(self):
        return self.get_output_port(self.motor_command_outport_index)


