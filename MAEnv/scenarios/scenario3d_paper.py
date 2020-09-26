# Environment # Reference #
# https://github.com/PX4/ecl/blob/master/geo/geo.cpp #
#  MAE - Aircraft [lon, lat, alt]
#  MAE - Entity   (x,y)=(0,0) 对应 (lat,lon) = (39.965376,116.325657)
#                 (z) === 0   对应 (alt) = 58.809239
#  lon/deg, lat/deg, alt/m, x/km, y/km, z/km


from abc import ABC
import numpy as np
import math
from MAControl.Util.Constrain import constrain
from MAEnv.core import World, Landmark
from MAEnv.scenario import BaseScenario
import MAEnv.scenarios.TargetProfile as T
from Mini0jsbsim.simulation import Simulation
import Mini0jsbsim.properties as prp


class Scenario(BaseScenario, ABC):
    def make_js_world(self, agent_num, target_type):
        world = World()
        # set agents = UAVs (in air)
        num_agents = agent_num
        world.agents = [Simulation() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.action_callback = True
        # set other entities (on ground)
        num_targets = T.num_targets
        num_grids = 5
        # add landmarks
        world.targets = [Landmark() for i in range(num_targets)]
        VALUE = [2, 10, 5]
        DEFENCE = [5, 1, 2]
        for i, landmark in enumerate(world.targets):
            landmark.name = 'target %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.value = VALUE[target_type[i]-1]
            landmark.size = T.target_size[i] * 0.01
            landmark.defence = DEFENCE[target_type[i]-1]
            landmark.attacking = False
            landmark.type = target_type[i]
        world.grids = [Landmark() for i in range(num_grids)]
        for i, landmark in enumerate(world.grids):
            landmark.name = 'grid %d' % i
            landmark.collide = False
            landmark.movable = False
            landmark.size = 0.005
            landmark.attacking = False
        world.landmarks = world.targets + world.grids
        # initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):
        ref_lat_deg = 39.965376   # 经度 (BIT)
        ref_lon_deg = 116.325657  # 纬度 (BIT)
        ref_alt_m = 58.809239     # 海拔 (BIT)

        for i, agent in enumerate(world.agents):
            agent.reinitialise()
            agent.color = T.agent_color
            agent.attacking = False
            agent.attacking_to = -1

        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_vel = np.zeros(world.dim_p)
            if i < len(world.targets):
                landmark.color = np.random.uniform(0, 1, 3)
                landmark.state.p_pos = np.array(T.target_pos[i])
                kkk = self.globallocalconverter(ref_lat_deg, ref_lon_deg, landmark.state.p_pos[0],
                                                landmark.state.p_pos[1], getxy=False)
                landmark.lat = round(kkk[0], 6)
                landmark.lon = round(kkk[1], 6)
                landmark.alt = ref_alt_m
            else:
                landmark.color = T.grid_color
                landmark.state.p_pos = np.array(T.grid_pos[i-len(world.targets)])

    def benchmark_data(self, agent, world):
        # returns data for benchmarking purposes
        rew = 0
        occupied_landmarks = 0
        min_dists = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            min_dists += min(dists)
            rew -= min(dists)
            if min(dists) < 0.1:
                occupied_landmarks += 1
        return rew, min_dists, occupied_landmarks

    def result(self, world):
        # TARGET-UAV分配情况
        res = []
        for a, agent in enumerate(world.agents):
            res.append(agent.attacking_to)  # 长度为UAV数量，每个元素是所攻击的目标（未攻击则为-1）
        res2 = []
        res3 = []
        for t in range(len(world.targets)):
            res2.append(res.count(t))  # 长度为TARGET数量，每个元素是分配给这个目标的UAV个数
            res3.append(world.targets[t].defence)  # 长度为TARGET数量，每个元素是这个目标需要的UAV个数
        res4 = list(np.array(res2)-np.array(res3))  # 长度为TARGET数量，每个元素是上述两项差值
        res5 = []   # 长度为TARGET数量，每个元素是根据差值计算出的奖励值
        for r in res4:
            if r >= 0:
                res5.append(1/(0.2*r+1))
            else:
                res5.append(0)
        return res5

    def reward(self, agent, world):
        # 根据TARGET-UAV分配情况，计算整体评价
        # 目前只考虑了效益，尚未考虑代价
        rew = 0
        res5 = self.result(world)
        for t, target in enumerate(world.targets):
            rew += target.value * res5[t]
        return rew

    @staticmethod
    def globallocalconverter(lat, lon, x, y, getxy):
        CONSTANTS_RADIUS_OF_EARTH = 6378137  # m # Equatorial
        ref_lat_rad = 39.965376 / 180 * math.pi
        ref_lon_rad = 116.325657 / 180 * math.pi
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        if getxy:
            lat_rad = lat * math.pi / 180
            lon_rad = lon * math.pi / 180
            sin_lat = math.sin(lat_rad)
            cos_lat = math.cos(lat_rad)
            cos_d_lon = math.cos(lon_rad - ref_lon_rad)
            arg = constrain(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
            c = math.acos(arg)
            k = 1.0
            if abs(c) > 0.0:
                k = c / math.sin(c)
            x = (k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH) / 1000
            y = (k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH) / 1000

        else:
            x_rad = x * 1000 / CONSTANTS_RADIUS_OF_EARTH
            y_rad = y * 1000 / CONSTANTS_RADIUS_OF_EARTH
            c = math.sqrt(x_rad * x_rad + y_rad * y_rad)
            if abs(c) > 0:
                sin_c = math.sin(c)
                cos_c = math.cos(c)
                lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
                lon_rad = (ref_lon_rad + math.atan2(y_rad * sin_c,
                                                    c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))
                lat = lat_rad / math.pi * 180
                lon = lon_rad / math.pi * 180
            else:
                lat = ref_lat_rad / math.pi * 180
                lon = ref_lon_rad / math.pi * 180

        return lat, lon, x, y

    def observation(self, agent, world):
        agent.obs = [agent.__getitem__(prp.altitude_sl_ft),
                     agent.__getitem__(prp.pitch_rad), agent.__getitem__(prp.roll_rad), agent.__getitem__(prp.heading_deg),
                     agent.__getitem__(prp.u_fps), agent.__getitem__(prp.v_fps), agent.__getitem__(prp.w_fps),
                     agent.__getitem__(prp.u_aero_fps), agent.__getitem__(prp.v_aero_fps), agent.__getitem__(prp.w_aero_fps),
                     agent.__getitem__(prp.v_north_fps), agent.__getitem__(prp.v_east_fps),
                     agent.__getitem__(prp.p_radps), agent.__getitem__(prp.q_radps), agent.__getitem__(prp.r_radps),
                     agent.__getitem__(prp.lat_geod_deg), agent.__getitem__(prp.lng_geoc_deg)]
        xy = list(self.globallocalconverter(agent.obs[15], agent.obs[16], 0.0, 0.0, True))[2:]
        agent.obs = agent.obs + xy
        # [0] altitude [1] pitch [2] roll [3] heading(yaw)
        # [4] u [5] v [6] w
        # [7] u-aero [8] v-aero [9] w-aero [10] v-north [11] v-east
        # [12] p [13] q [14] r
        # [15] lat [16] lon [17] 'x' [18] 'y'
        return agent.obs
