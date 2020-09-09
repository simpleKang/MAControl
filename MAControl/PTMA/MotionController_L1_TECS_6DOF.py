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
        self.roll_setpoint = 0.0
        self.nav_bearing = 0.0
        self.throttle_setpoint = 0.0

    def get_expected_action(self, obs, pointAi, pointBi, step, finishedi):

        # parameters update
        set_L1 = True
        if set_L1:
            L1_roll_limit = 0  # radians
            roll_slew_rate = 0  # .
            BP_range = 0.1  # (0.1km=100m)
        set_tecs = True
        if set_tecs:
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
        FLT_EPSILON = 0.0001

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
            self.tecs_update_pitch_throttle(obs, curr_wp[2], param_fw_thr_min, param_fw_thr_cruise)
        elif pos_sp_curr_type == 'loiter':
            loiter_radius = 10  # pos_sp_curr
            loiter_direction = [0, 1]  # pos_sp_curr
            if abs(loiter_radius) < FLT_EPSILON:
                loiter_radius = param_nav_loiter_rad = math.pi
                loiter_direction = 1
            self.l1_control_navigate_loiter(prev_wp, curr_wp, loiter_radius, loiter_direction, nav_speed_2d)
            att_sp_roll_body = self.roll_setpoint
            att_sp_yaw_body = self.nav_bearing
            alt_sp = pos_sp_curr_alt = 1000
            self.tecs_update_pitch_throttle(obs, curr_wp[2], param_fw_thr_min, param_fw_thr_cruise)
        else:
            pass

        return self.tangent_acc, self.lateral_acc, False

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

    def tecs_update_pitch_throttle(self, obs, sp_alt, thr_min, thr_cruise):
        CONSTANTS_ONE_G = 9.81551

        dt = self.world.dt
        throttle_setpoint_max = 0.95
        throttle_setpoint_min = thr_min
        pitch_setpoint_max = 0.9
        pitch_setpoint_min = -0.5
        max_climb_rate = 0
        max_sink_rate = 0
        min_sink_rate = 0
        climbout_mode_active = False

        # initialize states
        vert_vel_state = 0.0
        vert_pos_state = obs[0]
        tas_rate_state = 0.0
        TAS_setpoint_adj = TAS_setpoint_last = tas_state = obs[7:10]
        throttle_integ_state = 0.0
        pitch_integ_state = 0.0
        last_throttle_setpoint = thr_cruise
        pitch_setpoint_unc = last_pitch_setpoint = obs[1]
        hgt_setpoint_in_prev = hgt_setpoint_prev = hgt_setpoint_adj = hgt_setpoint_adj_prev = obs[0]
        underspeed_detected = False
        uncommanded_descent_recovery = False
        STE_rate_error = 0.0
        states_initialized = True

        # // Update the true airspeed state estimate
        TAS_setpoint = 0.7
        TAS_max = 0.9
        TAS_min = 0.2
        tas_error = 0.1  # 这里要改变一下记录连续变化
        tas_estimate_freq = 0.01
        tas_rate_state_input = tas_error * tas_estimate_freq * tas_estimate_freq
        if tas_state < 3.1:
            tas_rate_state_input = max(tas_rate_state_input, 0.0)
        tas_rate_state = tas_rate_state + tas_rate_state_input * dt
        speed_derivative = tas_error  # 这个可能有问题
        tas_state_input = tas_rate_state + speed_derivative + tas_error * tas_estimate_freq * 1.4142
        tas_state = tas_state + tas_state_input * dt
        tas_state = max(tas_state, 3.0)

        # 	// Calculate rate limits for specific total energy
        STE_rate_max = max_climb_rate * CONSTANTS_ONE_G
        STE_rate_min = min_sink_rate * CONSTANTS_ONE_G

        # 	// Detect an underspeed condition
        if tas_state < TAS_min * 0.9 and self.throttle_setpoint >= throttle_setpoint_max * 0.95:
            underspeed_detected = True
        else:
            underspeed_detected = False

        # 	// Detect an uncommanded descent caused by an unachievable airspeed demand
        # 	_detect_uncommanded_descent();
        #   // 似乎是循环进行相关步骤 这里才能有东西可用啊 迷惑 暂时放下

        # 	// Calculate the demanded true airspeed
        if underspeed_detected or uncommanded_descent_recovery:
            TAS_setpoint = TAS_min
        TAS_setpoint = constrain(TAS_setpoint, TAS_min, TAS_max)
        velRateMax = 0.5 * STE_rate_max / tas_state
        velRateMin = 0.5 * STE_rate_min / tas_state
        TAS_setpoint_adj = constrain(TAS_setpoint, TAS_min, TAS_max)
        speed_error_gain = 0.1
        TAS_rate_setpoint = (TAS_setpoint_adj - tas_state) * speed_error_gain
        TAS_rate_setpoint = constrain(TAS_rate_setpoint, velRateMin, velRateMax)

        # 	// Calculate the demanded height
        desired = hgt_setpoint = 100.0  # 尚且不清楚这是怎么来的
        if hgt_setpoint_in_prev < 0.1:
            hgt_setpoint_in_prev = desired
        hgt_setpoint = 0.5 * (desired + hgt_setpoint_in_prev)
        hgt_setpoint_in_prev = hgt_setpoint
        if (hgt_setpoint - hgt_setpoint_prev) > max_climb_rate * dt:
            hgt_setpoint = hgt_setpoint_prev + max_climb_rate * dt
        elif (hgt_setpoint - hgt_setpoint_prev) < max_sink_rate * dt:
            hgt_setpoint = hgt_setpoint_prev - max_sink_rate * dt
        hgt_setpoint_prev = hgt_setpoint
        hgt_setpoint_adj = 0.1 * hgt_setpoint + 0.9 * hgt_setpoint_adj_prev
        height_error_gain = 0.2
        height_setpoint_gain_ff = 0.02
        hgt_rate_setpoint = (hgt_setpoint_adj - obs[0]) * height_error_gain + height_setpoint_gain_ff * \
                            (hgt_setpoint_adj - hgt_setpoint_adj_prev) / dt
        hgt_setpoint_adj_prev = hgt_setpoint_adj
        if hgt_rate_setpoint > max_climb_rate:
            hgt_rate_setpoint = max_climb_rate
        elif hgt_rate_setpoint < -1 * max_sink_rate:
            hgt_rate_setpoint = -1 * max_sink_rate

        # 	// Calculate the specific energy values required by the control loop
        # specific energy demands in units of (m**2/sec**2) #
        SPE_setpoint = hgt_setpoint_adj * CONSTANTS_ONE_G  # potential #
        SKE_setpoint = 0.5 * TAS_setpoint_adj * TAS_setpoint_adj  # kinetic #
        # specific energy rate demands in units of (m**2/sec**3) #
        SPE_rate_setpoint = hgt_rate_setpoint * CONSTANTS_ONE_G
        SKE_rate_setpoint = tas_state * TAS_rate_setpoint
        # specific energies in units of (m**2/sec**2) #
        SPE_estimate = vert_pos_state * CONSTANTS_ONE_G
        SKE_estimate = 0.5 * tas_state * tas_state
        # specific energy rates in units of (m**2/sec**3) #
        SPE_rate = vert_vel_state * CONSTANTS_ONE_G
        SKE_rate = tas_state * speed_derivative

        # 	// Calculate the throttle demand
        STE_error = SPE_setpoint - SPE_estimate + SKE_setpoint - SKE_estimate
        STE_rate_setpoint = SPE_rate_setpoint + SKE_rate_setpoint
        STE_rate_setpoint = constrain(STE_rate_setpoint, STE_rate_min, STE_rate_max)
        STE_rate_error = 0.2 * (STE_rate_setpoint - SPE_rate - SKE_rate) + 0.8 * STE_rate_error
        if underspeed_detected:
            throttle_setpoint = 1.0
        else:
            load_factor = 1.0 / constrain(math.cos(obs[2]), 0.1, 1.0) - 1.0
            load_factor_correction = 0.2
            STE_rate_setpoint = STE_rate_setpoint + load_factor_correction * load_factor
        throttle_predicted = 0.0
        if STE_rate_setpoint >= 0:  # throttle is between cruise and maximum
            throttle_predicted = thr_cruise + STE_rate_setpoint / STE_rate_max * (throttle_setpoint_max - thr_cruise)
        else:  # throttle is between cruise and minimum
            throttle_predicted = thr_cruise + STE_rate_setpoint / STE_rate_min * (throttle_setpoint_min - thr_cruise)
        throttle_time_constant = 0.01
        throttle_damping_gain = 0.7
        STE_to_throttle = 1.0 / (throttle_time_constant * (STE_rate_max - STE_rate_min))
        throttle_setpoint = (STE_error + STE_rate_error * throttle_damping_gain) * STE_to_throttle + throttle_predicted
        throttle_setpoint = constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)
        last_throttle_setpoint = throttle_setpoint
        integrator_gain = 0.2
        if integrator_gain > 0.0:
            integ_state_max = throttle_setpoint_max - throttle_setpoint + 0.1
            integ_state_min = throttle_setpoint_min - throttle_setpoint - 0.1
            throttle_integ_state = throttle_integ_state + (STE_error * integrator_gain) * dt * STE_to_throttle
            if climbout_mode_active:
                throttle_integ_state = integ_state_max
            else:
                throttle_integ_state = constrain(throttle_integ_state, integ_state_min, integ_state_max)
        else:
            throttle_integ_state = 0.0
        throttle_setpoint = throttle_setpoint + throttle_integ_state
        throttle_setpoint = constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)

        # 	// Calculate the pitch demand
        SKE_weighting = 1.0  # 0.0 <= SKE_weighting <= 2.0
        if underspeed_detected or climbout_mode_active:
            SKE_weighting = 2.0
        SPE_weighting = 2.0 - SKE_weighting
        SEB_setpoint = SPE_setpoint * SPE_weighting - SKE_setpoint * SKE_weighting  # specific energy balance demand
        SEB_rate_setpoint = SPE_rate_setpoint * SPE_weighting - SKE_rate_setpoint * SKE_weighting
        SEB_error = SEB_setpoint - (SPE_estimate * SPE_weighting - SKE_estimate * SKE_weighting)
        SEB_rate_error = SEB_rate_setpoint - (SPE_rate * SPE_weighting - SKE_rate * SKE_weighting)
        pitch_time_constant = 0.2
        climb_angle_to_SEB_rate = tas_state * pitch_time_constant * CONSTANTS_ONE_G
        if integrator_gain > 0.0:
            pitch_integ_input = SEB_error * integrator_gain
            if pitch_setpoint_unc > pitch_setpoint_max:
                edge = (pitch_setpoint_max - pitch_setpoint_unc) * climb_angle_to_SEB_rate / pitch_time_constant
                pitch_integ_input = min(pitch_integ_input, min(edge, 0.0))
            elif pitch_setpoint_unc < pitch_setpoint_min:
                edge = (pitch_setpoint_min - pitch_setpoint_unc) * climb_angle_to_SEB_rate / pitch_time_constant
                pitch_integ_input = max(pitch_integ_input, max(edge, 0.0))
            pitch_integ_state = pitch_integ_state + pitch_integ_input * dt
        else:
            pitch_integ_state = 0.0
        pitch_damping_gain = 0.01
        SEB_correction = SEB_error + SEB_rate_error * pitch_damping_gain + SEB_rate_setpoint * pitch_time_constant
        if climbout_mode_active:
            SEB_correction += pitch_setpoint_min * climb_angle_to_SEB_rate
        pitch_setpoint_unc = (SEB_correction + pitch_integ_state) / climb_angle_to_SEB_rate
        pitch_setpoint = constrain(pitch_setpoint_unc, pitch_setpoint_min, pitch_setpoint_max)
        vert_accel_limit = 0.2
        ptchRateIncr = dt * vert_accel_limit / tas_state
        if pitch_setpoint - last_pitch_setpoint > ptchRateIncr:
            pitch_setpoint = last_pitch_setpoint + ptchRateIncr
        elif pitch_setpoint - last_pitch_setpoint < -1 * ptchRateIncr:
            pitch_setpoint = last_pitch_setpoint - ptchRateIncr
        last_pitch_setpoint = pitch_setpoint

        return [pitch_setpoint, throttle_setpoint]

    def l1_control_navigate_loiter(self, vectorA, vectorP, loiter_radius, loiter_direction, vectorVel):
        L1_period = 0.5
        L1_damping = 0.7
        L1_ratio = 0.2
        K_L1 = 0.1  # (系数)

        omega = 2.0 * math.pi / L1_period
        K_crosstrack = omega * omega
        K_velocity = 2.0 * L1_damping * omega

        # /* update bearing to next waypoint */ #
        vector_PA = vectorA - vectorP
        dist_PA = np.sqrt(vector_PA[0]*vector_PA[0] + vector_PA[1]*vector_PA[1])
        dist_PA = max(dist_PA, 0.000000001)
        target_bearing = vector_PA_unit = vector_PA / dist_PA

        nav_speed = np.sqrt(vectorVel[0]*vectorVel[0] + vectorVel[1]*vectorVel[1])
        nav_speed = max(nav_speed, 0.000000001)
        L1_distance = L1_ratio * nav_speed

        # /* calculate eta angle towards the loiter center */ #
        xtrack_vel_center = np.cross(-1 * vector_PA_unit, vectorVel)
        ltrack_vel_center = np.dot(vector_PA_unit, vectorVel)
        eta = math.atan2(xtrack_vel_center, ltrack_vel_center)
        eta = constrain(eta, -0.5 * math.pi, 0.5 * math.pi)

        lateral_accel_sp_center = K_L1 * nav_speed * nav_speed / L1_distance * math.sin(eta)
        xtrack_vel_circle = -1 * ltrack_vel_center
        xtrack_err_circle = dist_PA - loiter_radius
        crosstrack_error = xtrack_err_circle

        lateral_accel_sp_circle_pd = (xtrack_err_circle * K_crosstrack + xtrack_vel_circle * K_velocity)
        tangent_vel = xtrack_vel_center * loiter_direction
        if tangent_vel < 0.0:
            lateral_accel_sp_circle_pd = max(lateral_accel_sp_circle_pd, 0.0)

        max1 = max(0.5 * loiter_radius, loiter_radius + xtrack_err_circle)
        lateral_accel_sp_circle_centripetal = tangent_vel * tangent_vel / max1
        lateral_accel_sp_circle = loiter_direction * (lateral_accel_sp_circle_pd + lateral_accel_sp_circle_centripetal)

        # TBC

        return self.throttle_setpoint
