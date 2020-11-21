from MAControl.Base.InnerController import InnerController


class InnerController_PID(InnerController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(InnerController_PID, self).__init__(name, env, world, agent_index, arglist)
        self.ITerm_pitch = 0
        self.ITerm_roll = 0
        self.Error_pitch = 0
        self.Error_roll = 0

    def get_action(self, obs, pitch_sp, thr_sp, roll_sp, nav_bearing, step, finishedi):

        if finishedi:

            action = [0, 0, 0, 0, 0]

        else:

            delta_time = self.world.dt
            P_value = 0.4
            I_value = 0.0
            D_value = 0.0
            ITerm_window = 1

            error_pitch = pitch_sp - obs[1]
            delta_error_pitch = error_pitch - self.Error_pitch
            PTerm_pitch = error_pitch
            DTerm_pitch = delta_error_pitch / delta_time
            self.ITerm_pitch += error_pitch * delta_time
            if self.ITerm_pitch < -ITerm_window:
                self.ITerm_pitch = -ITerm_window
            elif self.ITerm_pitch > ITerm_window:
                self.ITerm_pitch = ITerm_window
            else:
                self.ITerm_pitch = self.ITerm_pitch
            self.Error_pitch = error_pitch

            error_roll = roll_sp - obs[2]
            delta_error_roll = error_roll - self.Error_roll
            PTerm_roll = error_roll
            DTerm_roll = delta_error_roll / delta_time
            self.ITerm_roll += error_roll * delta_time
            if self.ITerm_roll < -ITerm_window:
                self.ITerm_roll = -ITerm_window
            elif self.ITerm_roll > ITerm_window:
                self.ITerm_roll = ITerm_window
            else:
                self.ITerm_roll = self.ITerm_roll
            self.Error_roll = error_roll

            pitch_action = P_value * PTerm_pitch + I_value * self.ITerm_pitch + D_value * DTerm_pitch
            roll_action = P_value * PTerm_roll + I_value * self.ITerm_roll + D_value * DTerm_roll

            action = [-roll_action, roll_action, pitch_action, 0, thr_sp]

        return action

