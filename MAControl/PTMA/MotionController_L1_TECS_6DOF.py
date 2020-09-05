from MAControl.Base.MotionController import MotionController
import numpy as np
import math
from MAControl.Util.Constrain import constrain
# Flight Control # Reference #
# https://github.com/PX4/Firmware/blob/master/src/modules/fw_pos_control_l1/FixedwingPositionControl.cpp #
# https://github.com/PX4/Firmware/blob/master/src/lib/tecs/TECS.cpp #
# https://github.com/PX4/Firmware/blob/master/src/lib/l1/ECL_L1_Pos_Controller.cpp #


class MotionController_L1_TECS(MotionController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(MotionController_L1_TECS, self).__init__(name, env, world, agent_index, arglist)

        # extra params
        self.STE_rate_error = 0
        self.throttle_integ_s = 0
        self.tangent_acc = 0
        self.lateral_acc = 0

        # key values
        self.roll_setpoint = 0
        self.nav_bearing = 0

    def get_expected_action(self, obs, pointAi, pointBi, step, finishedi):

        # parameters update
        set_L1 = True
        if set_L1:
            L1_damping = 0
            L1_period = 0
            L1_roll_limit = 0  # radians
            roll_slew_rate = 0  # .
            BP_range = 0.1  # (0.1km=100m)
            K_L1 = 0.1  # (系数)
        set_tecs = True
        if set_tecs:
            max_climb_rate = 0
            max_sink_rate = 0
            speed_weight = 0
            indicated_airspeed_min = 0
            indicated_airspeed_max = 0
            time_const_throt = 0
            time_const = 0
            min_sink_rate = 0
            throttle_damp = 0
            integrator_gain = 0
            throttle_slewrate = 0
            vertical_accel_limit = 0
            speed_comp_filter_omega = 0
            roll_throttle_compensation = 0
            pitch_damping = 0
            heightrate_p = 0
            heightrate_ff = 0
            speedrate_p = 0
            TAS_setpoint = 0.05  # (km/s)
            throttle_c = 0  # (%)
            throttle_setpoint_max = 100  # (%)
            throttle_setpoint_min = 1  # (%)
            STE_rate_max = 0.025
            STE_rate_min = -0.025
            K_V = 1  # (系数)
            K_acct = 0.01  # (系数)
        param_fw_airspd_trim = 30
        param_fw_thr_min = 0.2
        param_fw_thr_cruise = 0.5

        # airspeed + attitude poll
        airspeed = obs[7:10]
        roll = obs[2]  # rad
        pitch = obs[1]  # rad
        yaw = obs[3]  # deg

        # get waypoint heading distance /lat /lon /alt
        setpoint_prev = pointAi[0:2]
        setpoint_next = pointBi[0:2]
        setpoint_hold = obs[15:17]
        setpoint_prev.append(obs[0])
        setpoint_next.append(obs[0])
        setpoint_hold.append(obs[0])

        # control position
        dt = self.world.dt
        setpoint = True
        att_sp_fw_control_yaw = False
        # # airspeed valid
        air_angle = math.atan2(obs[8], obs[7])
        ground_angle = math.atan2(obs[5], obs[4])
        ground_vel = np.sqrt(obs[4]*obs[4]+obs[5]*obs[5])
        if abs(air_angle - ground_angle) > math.pi / 2 or ground_vel < 3:
            nav_speed_2d = obs[7:9]
        else:
            nav_speed_2d = obs[4:6]
        throttle_max = 1
        was_in_air = True
        # # autonomous flight
        hold_alt = obs[0]
        hdg_hold_yaw = obs[3]
        was_circle_mode = False
        curr_wp = setpoint_hold[0:2]  # unsure
        prev_wp = setpoint_prev[0:2]
        next_wp = setpoint_next[0:2]
        att_sp_roll_reset_integral = False
        att_sp_pitch_reset_integral = False
        att_sp_yaw_reset_integral = False
        mission_airspeed = 30  # pos_sp_curr_cruising_speed
        mission_throttle = 0.7  # pos_sp_curr_cruising_throttle
        pos_sp_curr_type = 'position'
        if pos_sp_curr_type == 'idle':
            att_sp_thrust_body_0 = 0
            att_sp_roll_body = 0
            att_sp_pitch_body = 0
        elif pos_sp_curr_type == 'position':
            self.l1_control_navigate_waypoints(prev_wp, next_wp, curr_wp, nav_speed_2d)
            att_sp_roll_body = self.roll_setpoint
            att_sp_yaw_body = self.nav_bearing
            self.tecs_update_pitch_throttle(obs, curr_wp[2], param_fw_airspd_trim, param_fw_thr_min, param_fw_thr_cruise)



            att_sp_roll_body = 0.5  # l1 control
            att_sp_yaw_body = 0.5  # l1 control
            # tecs_update_pitch_throttle()
        elif pos_sp_curr_type == 'loiter':
            loiter_radius = 10  # pos_sp_curr
            loiter_direction = [0, 1]  # pos_sp_curr
            # l1_control_navigate_loiter()
            att_sp_roll_body = 0.5  # l1 control
            att_sp_yaw_body = 0.5  # l1 control
            # tecs_update_pitch_throttle()
            pass
        else:
            # copy thrust and pitch values from tecs
            # att_sp.thrust_body_0 = min(get_tecs_thrust(), throttle_max)
            pass






        # p-i-d
        Ki_STE = 0.01  # (系数)
        Kp_STE = 0.1   # (系数)
        Kd_STE = 0.0   # (系数)

        # if flag_init # then create waypoint from heading and distance
        # else # create waypoint from line and dist


        # set motion_pace
        motion_pace = 5
        if step == 0 or step % motion_pace == 0:

            use_tecs_pitch = True
            if use_tecs_pitch:
                pass
                # att_sp_pitch_body = get_tecs_pitch() # tecs_get_pitch_setpoint

            # tecs_get_throttle_setpoint

            curr_lat = obs[15]
            curr_lon = obs[16]
            curr_alt = obs[0]

            param_fw_rsp_off = 0.2  # radians
            param_fw_psp_off = 0.2  # radians

            att_sp_roll_body = att_sp_roll_body + param_fw_rsp_off
            att_sp_pitch_body = att_sp_pitch_body + param_fw_psp_off
            att_sp_q_d = [att_sp_roll_body, att_sp_pitch_body, att_sp_yaw_body]

            # # # # # tecs # # # # #

            # compute rate setpoints
            tas_state = speed = np.sqrt(np.square(airspeed[0]) + np.square(airspeed[1]))
            TAS_rate_setpoint = (TAS_setpoint - tas_state) * K_V
            STE_error = 0.5 * (TAS_setpoint * TAS_setpoint - tas_state * tas_state)
            STE_rate_setpoint = constrain(tas_state * TAS_rate_setpoint, STE_rate_min, STE_rate_max)

            # compute throttle_p
            if STE_rate_setpoint >= 0:
                throttle_p = throttle_c + STE_rate_setpoint / STE_rate_max * (throttle_setpoint_max - throttle_c)
            else:
                throttle_p = throttle_c + STE_rate_setpoint / STE_rate_min * (throttle_setpoint_min - throttle_c)

            # compute throttle_setpoint
            self.STE_rate_error = self.STE_rate_error * 0.8 + STE_rate_setpoint * 0.2
            self.throttle_integ_s = self.throttle_integ_s + STE_error * Ki_STE
            throttle_setpoint = throttle_p + (STE_error + self.STE_rate_error * Kd_STE) * Kp_STE + self.throttle_integ_s
            throttle_setpoint = constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)

            # tangent_acc
            self.tangent_acc = throttle_setpoint * K_acct

        if finishedi:
            arrive_flag = True
            self.tangent_acc = 0
            self.lateral_acc = 0
        else:
            dist_BP = 1
            if dist_BP < BP_range:
                arrive_flag = True
            else:
                arrive_flag = False

        return self.tangent_acc, self.lateral_acc, arrive_flag

    def l1_control_navigate_waypoints(self, vectorA, vectorB, vectorP, vectorVel):
        L1_ratio = 0.1  # (当v=0.05则L1=0.005km=50m)

        # /* this follows the logic presented in [1] */ #
        eta = 0.0
        xtrack_vel = 0.0
        ltrack_vel = 0.0

        # /* get the direction between the last (visited) and next waypoint */ #
        vector_PB = vectorB - vectorP
        dist_PB = np.sqrt(vector_PB[0]*vector_PB[0] + vector_PB[1]*vector_PB[1])
        dist_PB = max(dist_PB, 0.000000001)
        target_bearing = vector_PB_unit = vector_PB / dist_PB

        nav_speed = np.sqrt(vectorVel[0]*vectorVel[0] + vectorVel[1]*vectorVel[1])
        nav_speed = max(nav_speed, 0.000000001)
        L1_distance = L1_ratio * nav_speed

        # /* calculate vector from A to B */ #
        vector_AB = vectorB - vectorA
        dist_AB = np.sqrt(vector_AB[0]*vector_AB[0] + vector_AB[1]*vector_AB[1])
        dist_AB = max(dist_AB, 0.000000001)
        vector_AB_unit = vector_AB / dist_AB

        # /* calculate the vector from waypoint A to the aircraft */ #
        vector_AP = vectorP - vectorA
        dist_AP = np.sqrt(vector_AP[0]*vector_AP[0] + vector_AP[1]*vector_AP[1])
        dist_AP = max(dist_AP, 0.000000001)
        vector_AP_unit = vector_AP / dist_AP

        crosstrack_error = np.cross(vector_AB_unit, vector_AP)
        alongTrackDist = np.dot(vector_AB_unit, vector_AP)

        # /* calculate angle of airplane position vector relative to line) */ #
        AB_to_BP_bearing = math.atan2(np.cross(-1*vector_PB_unit, vector_AB_unit),
                                      np.dot(-1*vector_PB_unit, vector_AB_unit))

        if dist_AP > L1_distance and alongTrackDist / dist_AP < -0.707:

            # calculate eta to fly to waypoint A
            xtrack_vel = np.cross(vectorVel, -1*vector_AP_unit)
            ltrack_vel = np.dot(vectorVel, -1*vector_AP_unit)
            eta = math.atan2(xtrack_vel, ltrack_vel)
            self.nav_bearing = math.atan2(-1*vector_AP_unit[1], -1*vector_AP_unit[0])
            # If the AB vector and the vector from B to airplane point in the same direction,
            # we have missed the waypoint. At +- 90 degrees we are just passing it.

        elif abs(AB_to_BP_bearing) < math.radians(100):

            # calculate eta to fly to waypoint B
            xtrack_vel = np.cross(vectorVel, vector_PB_unit)
            ltrack_vel = np.dot(vectorVel, vector_PB_unit)
            eta = math.atan2(xtrack_vel, ltrack_vel)
            self.nav_bearing = math.atan2(vector_PB_unit[1], vector_PB_unit[0])

        else:

            # calculate eta to fly along the line between A and B
            xtrack_vel = np.cross(vectorVel, vector_AB_unit)
            ltrack_vel = np.dot(vectorVel, vector_AB_unit)
            eta2 = math.atan2(xtrack_vel, ltrack_vel)
            xtrackErr = np.cross(vector_AP, vector_AB)
            eta1 = math.asin(constrain(xtrackErr / L1_distance, -0.7071, 0.7071))
            eta = eta1 + eta2
            self.nav_bearing = math.atan2(vector_AB[1], vector_AB[0]) + eta1
            # bearing from current position to L1 point

        eta = constrain(eta, -1*math.pi/2, math.pi/2)
        lateral_accel = nav_speed * nav_speed / L1_distance * math.sin(eta)
        circle_mode = False
        bearing_error = eta

        # update roll setpoint
        roll_new = math.atan(lateral_accel * 1.0 / 9.81551)
        self.roll_setpoint = roll_new

        return None

    def tecs_update_pitch_throttle(self, obs, sp_alt, airspd_trim, thr_min, thr_cruise):

        dt = self.world.dt
        throttle_setpoint_max = 0.95
        throttle_setpoint_min = thr_min
        pitch_setpoint_max = 0.9
        pitch_setpoint_min = -0.5

        # initialize states
        vert_vel_state = 0.0
        vert_pos_state = obs[0]
        tas_rate_state = 0.0
        TAS_setpoint_adj = TAS_setpoint_last = tas_state = obs[7:10]
        throttle_integ_state = 0.0
        pitch_integ_state = 0.0
        last_throttle_setpoint = thr_cruise
        pitch_setpoint_unc = last_pitch_setpoint = obs[1]
        underspeed_detected = False
        uncommanded_descent_recovery = False
        STE_rate_error = 0.0
        states_initialized = True

        # // Update the true airspeed state estimate
        # 	_update_speed_states(EAS_setpoint, indicated_airspeed, eas_to_tas);
        #
        # 	// Calculate rate limits for specific total energy
        # 	_update_STE_rate_lim();
        #
        # 	// Detect an underspeed condition
        # 	_detect_underspeed();
        #
        # 	// Detect an uncommanded descent caused by an unachievable airspeed demand
        # 	_detect_uncommanded_descent();
        #
        # 	// Calculate the demanded true airspeed
        # 	_update_speed_setpoint();
        #
        # 	// Calculate the demanded height
        # 	_update_height_setpoint(hgt_setpoint, baro_altitude);
        #
        # 	// Calculate the specific energy values required by the control loop
        # 	_update_energy_estimates();
        #
        # 	// Calculate the throttle demand
        # 	_update_throttle_setpoint(throttle_cruise, rotMat);
        #
        # 	// Calculate the pitch demand
        # 	_update_pitch_setpoint();
        #
        # 	// Update time stamps
        # 	_pitch_update_timestamp = now;




        hrt_abstime = 0
        alt_sp = 0
        airspeed_sp = 0
        pitch_min_rad = 0
        pitch_max_rad = 0
        throttle_min = 0
        throttle_max = 0
        throttle_cruise = 0
        climbout_mode = 0
        climbout_pitch_min_rad = 0
        mode = 0



        param_fw_rsp_off = 0.2  # radians
        param_fw_psp_off = 0.2  # radians
        pitch = 0

        pitch_for_tecs = pitch - param_fw_psp_off

        return None

