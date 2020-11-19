from MAControl.Base.MotionController import MotionController
import numpy as np
import math
from MAControl.Util.Constrain import constrain

# >>>>> Flight Control >>>>>> Reference
# https://github.com/PX4/Firmware/blob/master/src/modules/fw_pos_control_l1/FixedwingPositionControl.cpp #
# https://github.com/PX4/Firmware/blob/master/src/lib/tecs/TECS.cpp #
# https://github.com/PX4/Firmware/blob/master/src/lib/l1/ECL_L1_Pos_Controller.cpp #


class MotionController_L1_TECS(MotionController):

    def __init__(self, name, env, world, agent_index, arglist):
        super(MotionController_L1_TECS, self).__init__(name, env, world, agent_index, arglist)
        self.circle_mode = False
        self.throttle_setpoint = 0.0
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.nav_bearing = 0.0

        self.vel_last = 0.0
        self.tas_rate_state_last = 0.0
        self.STE_rate_error_last = 0.0
        self.throttle_integ_state_last = 0.0
        self.pitch_integ_state_last = 0.0

    def get_expected_action(self, obs, pointAi, pointBi, step, finishedi):

        # parameters update
        param_fw_airspd_min = 400  # fps
        param_fw_airspd_max = 600  # fps
        airspeed_demand = 300  # fps
        param_fw_thr_min = 0.2
        param_fw_thr_cruise = 0.5
        throttle_max = 1.0
        param_fw_p_lim_min = 0.15
        param_fw_p_lim_max = 0.75
        mission_airspeed = 300  # pos_sp_curr_cruising_speed (fps)
        mission_throttle = 0.7  # pos_sp_curr_cruising_throttle
        acc_rad = 0.5  # l1_control_switch_distance

        # target_airspeed ( no countering for pushed more and more away by wind
        n = math.sqrt(math.cos(obs[2]))
        adjusted_min_airspeed = constrain(param_fw_airspd_min / n, param_fw_airspd_min, param_fw_airspd_max)
        target_airspeed = constrain(airspeed_demand, adjusted_min_airspeed, param_fw_airspd_max)

        # control position ( curr_pos / ground_speed / pos_sp_prev / pos_sp_curr / pos_sp_next (x)
        nav_speed_2d = ground_speed = obs[10:12]
        curr_pos = obs[17:19]
        pos_sp_prev = pointAi
        pos_sp_curr = pointBi
        # —— —— —— autonomous flight —— —— —— #
        hold_alt = obs[0]
        hdg_hold_yaw = obs[3]
        was_circle_mode = False
        curr_wp = pos_sp_curr
        prev_wp = pos_sp_prev
        att_sp_roll_reset_integral = False
        att_sp_pitch_reset_integral = False
        att_sp_yaw_reset_integral = False
        pos_sp_curr_type = 'position'  # ( no position_sp_type to "loiter" yet
        if pos_sp_curr_type == 'idle':
            att_sp_thrust_body_0 = 0
            att_sp_roll_body = 0
            att_sp_pitch_body = 0
        elif pos_sp_curr_type == 'position':
            self.l1_control_navigate_waypoints(prev_wp, curr_wp, curr_pos, nav_speed_2d)
            att_sp_roll_body = self.roll_setpoint
            att_sp_yaw_body = self.nav_bearing
            self.tecs_update_pitch_throttle(obs, obs[0], target_airspeed, param_fw_p_lim_min, param_fw_p_lim_max,
                                            param_fw_thr_min, throttle_max, param_fw_thr_cruise)
        elif pos_sp_curr_type == 'loiter':
            # ( not in use yet
            pass
        else:
            pass
        if was_circle_mode and (not self.circle_mode):
            att_sp_roll_reset_integral = True
        # —— —— —— copy thrust output for publication  —— —— —— #
        att_sp_thrust_body_0 = min(self.throttle_setpoint, throttle_max)
        att_sp_pitch_body = self.pitch_setpoint

        # >>>>>>> FixedwingPositionControl::Run() # Procedure # ( DO NOT DELETE PLZ )
        # parameter_update_s pupdate;
        # _parameter_update_sub.copy(&pupdate);
        # parameters_update();
        # vehicle_global_position_s gpos;
        # _current_latitude = gpos.lat;
        # _current_longitude = gpos.lon;
        # _current_altitude = -_local_pos.z + _local_pos.ref_alt;
        # _hdg_hold_enabled = false;
        # _alt_reset_counter = _local_pos.vz_reset_counter;
        # _pos_reset_counter = _local_pos.vxy_reset_counter;
        # airspeed_poll();
        # _pos_sp_triplet_sub.update(&_pos_sp_triplet);
        # vehicle_attitude_poll();
        # vehicle_control_mode_poll();
        # _vehicle_status_sub.update(&_vehicle_status);
        # _vehicle_acceleration_sub.update();
        # _vehicle_rates_sub.update();
        # Vector2f curr_pos((float)_current_latitude, (float)_current_longitude);
        # Vector2f ground_speed(_local_pos.vx, _local_pos.vy);
        # // Convert Local setpoints to global setpoints.
        # _att_sp.roll_body += radians(_param_fw_rsp_off.get());
        # _att_sp.pitch_body += radians(_param_fw_psp_off.get());
        # const Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
        # q.copyTo(_att_sp.q_d);
        # _att_sp.timestamp = hrt_absolute_time();
        # _attitude_sp_pub.publish(_att_sp);
        # status_publish();

        return [self.pitch_setpoint, self.throttle_setpoint, self.roll_setpoint, self.nav_bearing]

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
        vector_AB = np.array(vectorB) - np.array(vectorA)
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
        self.circle_mode = False
        bearing_error = eta

        # update roll setpoint
        roll_new = math.atan(lateral_accel * 1.0 / 9.81551)
        self.roll_setpoint = roll_new

        return None

    def tecs_update_pitch_throttle(self, obs, alt_sp, airspeed_sp, pitch_min_rad, pitch_max_rad,
                                   throttle_min, throttle_max, thr_cruise):

        CONSTANTS_ONE_G = 9.81551
        max_climb_rate = min_sink_rate = 0.5
        # // Set class variables from inputs
        dt = self.world.dt
        throttle_setpoint_max = throttle_max
        throttle_setpoint_min = throttle_min
        pitch_setpoint_max = pitch_max_rad
        pitch_setpoint_min = pitch_min_rad
        vel = obs[4:7]

        # // initialize states
        vert_vel_state = obs[6]*(-1)
        vert_pos_state = obs[0]
        TAS_setpoint_adj = TAS_setpoint_last = tas_state = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
        pitch_setpoint_unc = last_pitch_setpoint = constrain(obs[1], pitch_setpoint_min, pitch_setpoint_max)
        hgt_setpoint_in_prev = hgt_setpoint_prev = hgt_setpoint_adj = hgt_setpoint_adj_prev = obs[0]
        underspeed_detected = False
        uncommanded_descent_recovery = False
        states_initialized = True

        # // Update the true airspeed state estimate
        TAS_setpoint = airspeed_sp
        TAS_max = 600
        TAS_min = 400
        tas_error = tas_state - self.vel_last
        self.vel_last = tas_state
        tas_estimate_freq = 0.01
        tas_rate_state_input = tas_error * tas_estimate_freq * tas_estimate_freq
        if tas_state < 3.1:
            tas_rate_state_input = max(tas_rate_state_input, 0.0)
        tas_rate_state = self.tas_rate_state_last + tas_rate_state_input * dt
        self.tas_rate_state_last = tas_rate_state
        speed_derivative = tas_error  # 近似
        tas_state_input = tas_rate_state + speed_derivative + tas_error * tas_estimate_freq * 1.4142
        tas_state = tas_state + tas_state_input * dt
        tas_state = max(tas_state, 3.0)

        # 	// Calculate rate limits for specific total energy
        STE_rate_max = max_climb_rate * CONSTANTS_ONE_G
        STE_rate_min = -min_sink_rate * CONSTANTS_ONE_G

        # 	// Detect underspeed condition >>  Detect uncommanded descent caused by unachievable airspeed demand (x)
        if (tas_state < TAS_min * 0.9 and self.throttle_setpoint >= throttle_setpoint_max * 0.95)\
                or (vert_pos_state < hgt_setpoint_adj and underspeed_detected):
            underspeed_detected = True
        else:
            underspeed_detected = False

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
        hgt_rate_setpoint = 0.0

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
        STE_rate_error = 0.2 * (STE_rate_setpoint - SPE_rate - SKE_rate) + 0.8 * self.STE_rate_error_last
        self.STE_rate_error_last = STE_error
        if underspeed_detected:
            self.throttle_setpoint = 1.0
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
        self.throttle_setpoint = constrain(throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)
        integrator_gain = 0.2
        if integrator_gain > 0.0:
            integ_state_max = throttle_setpoint_max - self.throttle_setpoint + 0.1
            integ_state_min = throttle_setpoint_min - self.throttle_setpoint - 0.1
            throttle_integ_state = self.throttle_integ_state_last + (STE_error * integrator_gain) * dt * STE_to_throttle
            throttle_integ_state = constrain(throttle_integ_state, integ_state_min, integ_state_max)
        else:
            throttle_integ_state = 0.0
        self.throttle_integ_state_last = throttle_integ_state
        self.throttle_setpoint = self.throttle_setpoint + throttle_integ_state
        self.throttle_setpoint = constrain(self.throttle_setpoint, throttle_setpoint_min, throttle_setpoint_max)

        # 	// Calculate the pitch demand
        SKE_weighting = 1.0  # 0.0 <= SKE_weighting <= 2.0 ( pitch speed weight
        if underspeed_detected:
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
            pitch_integ_state = self.pitch_integ_state_last + pitch_integ_input * dt
        else:
            pitch_integ_state = 0.0
        self.pitch_integ_state_last = pitch_integ_state
        pitch_damping_gain = 0.01
        SEB_correction = SEB_error + SEB_rate_error * pitch_damping_gain + SEB_rate_setpoint * pitch_time_constant
        pitch_setpoint_unc = (SEB_correction + pitch_integ_state) / climb_angle_to_SEB_rate
        pitch_setpoint = constrain(pitch_setpoint_unc, pitch_setpoint_min, pitch_setpoint_max)
        vert_accel_limit = 0.2
        ptchRateIncr = dt * vert_accel_limit / tas_state
        if pitch_setpoint - self.pitch_setpoint > ptchRateIncr:
            pitch_setpoint = self.pitch_setpoint + ptchRateIncr
        elif pitch_setpoint - self.pitch_setpoint < -1 * ptchRateIncr:
            pitch_setpoint = self.pitch_setpoint - ptchRateIncr
        else:
            pass
        self.pitch_setpoint = pitch_setpoint

        return [self.pitch_setpoint, self.throttle_setpoint]

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

        # /* Switch between circle (loiter) and capture (towards waypoint center) mode when
        # 	 * the commands switch over. Only fly towards waypoint if outside the circle. */
        if (lateral_accel_sp_center < lateral_accel_sp_circle and loiter_direction > 0.0 and xtrack_err_circle > 0.0) \
                or \
           (lateral_accel_sp_center > lateral_accel_sp_circle and loiter_direction < 0.0 and xtrack_err_circle < 0.0):
            lateral_accel = lateral_accel_sp_center
            self.circle_mode = False
            bearing_error = eta  # angle between requested and current velocity vector
            nav_bearing = math.atan2(vector_PA_unit[1], vector_PA_unit[0])  # bearing from current position to L1 point
        else:
            lateral_accel = lateral_accel_sp_circle
            self.circle_mode = True
            bearing_error = 0.0
            nav_bearing = math.atan2(vector_PA_unit[1], vector_PA_unit[0])  # bearing from current position to L1 point

        # update roll setpoint
        roll_new = math.atan(lateral_accel * 1.0 / 9.81551)
        self.roll_setpoint = roll_new

        return None
