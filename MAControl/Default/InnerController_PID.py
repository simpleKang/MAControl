from MAControl.Base.InnerController import InnerController
import numpy as np


class InnerController_PID(InnerController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(InnerController_PID, self).__init__(name, env, world, agent_index, arglist)

        # extra params
        self.ITerm = 0
        self.last_error = 0

    def get_action(self, obs, Eacct, Eaccl, step, finishedi):
        # print('inner control')

        if finishedi:

            action = [0, 0, 0, 0, 0]

        else:

            Exp_lateral_acc = Eaccl
            True_lateral_acc = round(obs[5], 5)

            delta_time = self.world.dt
            P_value = 0.8
            I_value = 0.08
            D_value = 0.0
            Iterm_window = 1

            error = Exp_lateral_acc - True_lateral_acc
            delta_error = error - self.last_error
            PTerm = error
            DTerm = delta_error / delta_time
            self.ITerm += error * delta_time
            if self.ITerm < -Iterm_window:
                self.ITerm = -Iterm_window
            elif self.ITerm > Iterm_window:
                self.ITerm = Iterm_window
            else:
                self.ITerm = self.ITerm
            self.last_error = error

            acct = Eacct
            accl = P_value * PTerm + I_value * self.ITerm + D_value * DTerm
            vel_vector = np.array(obs[0:2])
            vel_vector[0] = round(vel_vector[0], 4)
            vel_vector[1] = round(vel_vector[1], 4)
            speed = np.sqrt(np.square(vel_vector[0]) + np.square(vel_vector[1]))
            vel_right_unit = np.array([vel_vector[1], -1 * vel_vector[0]]) / speed
            acc = acct * vel_vector / speed + accl * vel_right_unit

            action = [0, acc[0], 0, acc[1], 0]

        return action

