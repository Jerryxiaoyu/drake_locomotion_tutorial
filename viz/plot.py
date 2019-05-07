from matplotlib.pylab import plt
import numpy as np


class PlotResults(object):
    def __init__(self, states_log, states_desired_log, command_log,  timestep):
        self.states_log = states_log
        self.states_desired_log = states_desired_log
        self.command_log = command_log
        self.timestep = timestep
        self.t = np.linspace(0, states_log.shape[1] * timestep, num=states_log.shape[1])

    def plot_single_leg_curves(self, leg_no):
        t = self.t
        q_h_d = self.states_desired_log[0 + leg_no - 1]
        q_t_d = self.states_desired_log[4 + leg_no - 1]
        q_c_d = self.states_desired_log[8 + leg_no - 1]

        q_h = self.states_log[7 + leg_no - 1]
        q_t = self.states_log[11 + leg_no - 1]
        q_c = self.states_log[15 + leg_no - 1]

        c_h = self.command_log[3 * (leg_no - 1)]
        c_t = self.command_log[3 * (leg_no - 1) + 1]
        c_c = self.command_log[3 * (leg_no - 1) + 2]

        # c_h_ff = ff_command_data[3 * (leg_no - 1)]
        # c_t_ff = ff_command_data[3 * (leg_no - 1) + 1]
        # c_c_ff = ff_command_data[3 * (leg_no - 1) + 2]

        plt.subplot(121)

        plt.plot(t, q_h, 'b', label='hip')
        plt.plot(t, q_t, 'g', label='thigh')
        plt.plot(t, q_c, 'r', label='calf')
        plt.legend()

        plt.plot(t, q_h_d, '--b', label='hip')
        plt.plot(t, q_t_d, '--g', label='thigh')
        plt.plot(t, q_c_d, '--r', label='calf')

        plt.ylim([-1.8, 1.5])

        plt.subplot(121)
        plt.plot(t, c_h, 'b', label='hip')
        plt.plot(t, c_t, 'g', label='thigh')
        plt.plot(t, c_c, 'r', label='calf')
        plt.legend()

        # plt.plot(t, c_h_ff, '--b', label='hip')
        # plt.plot(t, c_t_ff, '--g', label='thigh')
        # plt.plot(t, c_c_ff, '--r', label='calf')