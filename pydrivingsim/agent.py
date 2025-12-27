# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT

import ctypes as ct
from datetime import datetime
from math import *

import agent.agent_interfaces_connector as agent_lib
from agent.interfaces_python_data_structs import input_data_str, output_data_str

from pydrivingsim import World, Vehicle, TrafficLight, TrafficCone, Target, SuggestedSpeedSignal, Coin


c = agent_lib.AgentConnector()

# Define types
type_integer_4 = ct.c_int32 * 4
type_integer_10 = ct.c_int32 * 10
type_integer_20 = ct.c_int32 * 20
type_double_10 = ct.c_double * 10
type_double_20 = ct.c_double * 20
type_double_100 = ct.c_double * 100

class Agent():
    __metadata = {
        "dt": 0.05
    }
    def __init__(self, vehicle: Vehicle, track=None):
        c.test_lib()
        self.vehicle = vehicle
        self.track = track
        self._last_s = None
        self._search_window = 50.0

        # Simulation information
        assert self.__metadata["dt"] >= World().get_dt()
        self.sim_call_freq = self.__metadata["dt"] / World().get_dt()
        self.num_of_step = 0

        IP = [127, 0, 0, 1]  # Remote usage
        PORT = 30000  # Default port for agent_test
        log_enable = 0  # The log can be True only in local usage
        type_of_log = 0  # Different level of log

        # Function for agent_test initialization
        c.client_agent_init_num(type_integer_4(*IP), PORT, log_enable, type_of_log)

        self.scenario_msg = input_data_str()
        self.manoeuvre_msg = output_data_str()

        # Create the pointer for the function call
        self.scenario_msg_pointer = ct.pointer(self.scenario_msg)
        self.manoeuvre_msg_pointer = ct.pointer(self.manoeuvre_msg)

        self.cycle_number = 0
        self.requested_cruising_speed = 20
        self.action = (0,0)

        # Data to be filtered
        self.ALgtFild = 0
        self.YawRateFild = 0
        self.SteerWhlAg = 0

    def compute(self):
        self.num_of_step += 1
        self.__filtering(self.vehicle)
        if self.num_of_step >= self.sim_call_freq:
            self.__compute(self.vehicle)
            self.num_of_step = 0
            self.__clear_filter()


    def __filtering(self, v :Vehicle):
        s: input_data_str = self.scenario_msg
        self.ALgtFild += v.dX[3] - v.state[5] * v.state[4]
        self.YawRateFild += v.state[5]
        self.SteerWhlAg += v.state[10]

    def __clear_filter(self):
        s: input_data_str = self.scenario_msg
        self.ALgtFild = 0
        self.YawRateFild = 0
        self.SteerWhlAg = 0

    def __compute(self, v :Vehicle):
        s :input_data_str = self.scenario_msg
        m :output_data_str = self.manoeuvre_msg

        # Closing env if agent request to close
        if m.Status == 1:
            self.terminate()

        # Basic parameters
        self.cycle_number += 1
        s.CycleNumber = self.cycle_number
        s.ID = 0
        s.TimeStamp = ct.c_double(datetime.timestamp(datetime.now()))
        s.Status = 0
        s.ECUupTime = World().time

        # Vehicle parameters
        s.VehicleLen = v.vehicle.vehicle.L                          # double - lenght dimension [m]
        s.VehicleWidth = v.vehicle.vehicle.Wf                       # double - width dimension [m]
        s.LaneHeading = -v.state[2]
        #print(v.state[2])
        #print((v.state[0],v.state[1]))
        s.VLgtFild = v.state[3]
        s.ALgtFild = self.ALgtFild/self.num_of_step
        s.YawRateFild = self.YawRateFild/self.num_of_step
        s.SteerWhlAg = self.SteerWhlAg/self.num_of_step
        s.RequestedCruisingSpeed = self.requested_cruising_speed

        if self.track is not None:
            road_width = self.track.track_width
            s.LaneWidth = road_width

            # Use track geometry for consistent heading/lateral errors.
            if self._last_s is None:
                s_proj, lateral_error, track_heading = self.track.project(
                    v.state[0], v.state[1]
                )
            else:
                s_proj, lateral_error, track_heading = self.track.project(
                    v.state[0], v.state[1],
                    s_hint=self._last_s,
                    search_window=self._search_window
                )
            self._last_s = s_proj

            heading_error = track_heading - v.state[2]
            heading_error = atan2(sin(heading_error), cos(heading_error))

            s.LaneHeading = heading_error
            s.LaneCrvt = self.track.curvature(s_proj)

            # Keep legacy sign convention used by the C++ controller:
            # lane offsets average should be positive when vehicle is right of center.
            s.LatOffsLineR = -lateral_error - road_width / 2
            s.LatOffsLineL = -lateral_error + road_width / 2
        else:
            road_width = 4
            s.LaneWidth = road_width

            # Lateral Position - compute from coins (track centerline markers)
            # Find two nearest coins: one behind and one ahead to interpolate centerline
            coins_ahead = []
            coins_behind = []
            for obj in World().obj_list:
                if type(obj) is Coin:
                    delta_x = obj.pos[0] - v.state[0]
                    if delta_x >= 0:
                        coins_ahead.append((delta_x, obj.pos[0], obj.pos[1]))
                    else:
                        coins_behind.append((abs(delta_x), obj.pos[0], obj.pos[1]))

            # Sort by distance and get nearest
            coins_ahead.sort(key=lambda c: c[0])
            coins_behind.sort(key=lambda c: c[0])

            if len(coins_ahead) >= 1 and len(coins_behind) >= 1:
                # Interpolate between coins to get reference point and heading
                coin_behind = coins_behind[0]  # (dist, x, y)
                coin_ahead = coins_ahead[0]

                # Interpolate to find centerline y at vehicle x position
                t = coin_behind[0] / (coin_behind[0] + coin_ahead[0]) if (coin_behind[0] + coin_ahead[0]) > 0 else 0.5
                ref_y = coin_behind[2] + t * (coin_ahead[2] - coin_behind[2])

                # Compute heading of centerline between coins
                dx = coin_ahead[1] - coin_behind[1]
                dy = coin_ahead[2] - coin_behind[2]
                lane_heading_world = atan2(dy, dx)

                # Lateral error: positive = vehicle is to the left of centerline
                lateral_error = v.state[1] - ref_y

                # Lane heading relative to vehicle (positive = lane turns left relative to vehicle)
                heading_error = lane_heading_world - v.state[2]
                # Wrap to [-pi, pi]
                heading_error = atan2(sin(heading_error), cos(heading_error))

                s.LaneHeading = heading_error
                s.LatOffsLineR = -lateral_error - road_width / 2
                s.LatOffsLineL = -lateral_error + road_width / 2
            elif len(coins_ahead) >= 2:
                # Use two coins ahead
                coin1 = coins_ahead[0]
                coin2 = coins_ahead[1]
                dx = coin2[1] - coin1[1]
                dy = coin2[2] - coin1[2]
                lane_heading_world = atan2(dy, dx)
                heading_error = lane_heading_world - v.state[2]
                heading_error = atan2(sin(heading_error), cos(heading_error))
                lateral_error = v.state[1] - coin1[2]
                s.LaneHeading = heading_error
                s.LatOffsLineR = -lateral_error - road_width / 2
                s.LatOffsLineL = -lateral_error + road_width / 2
            else:
                # Fallback: straight road
                s.LaneHeading = -v.state[2]
                s.LatOffsLineR = -v.state[1] - road_width / 2
                s.LatOffsLineL = -v.state[1] + road_width / 2

        # Objects parameters (traffic light and obstacles)
        trafficlight = 0
        trafficlightDist = 0
        speedlimitId = 0
        objId = 0
        target = 0
        for obj in World().obj_list:
            if type(obj) is TrafficLight:
                # Get the closest trafficlight in front
                if trafficlightDist == 0 or (obj.pos[0] - v.state[0] > -1.0 and obj.pos[0] - v.state[0] < trafficlightDist):
                    trafficlightDist = obj.pos[0] - v.state[0]
                    trafficlight = obj

            if type(obj) is TrafficCone and objId < 20:
                s.ObjID[objId] = 1
                delta_x = obj.pos[0] - v.state[0]
                delta_y = obj.pos[1] - v.state[1]
                s.ObjX[objId] = delta_x * cos(v.state[2]) + delta_y * sin(v.state[2])
                s.ObjY[objId] = - delta_x * sin(v.state[2]) + delta_y * cos(v.state[2])
                s.ObjVel[objId] = 0
                s.ObjLen[objId] = obj.size
                s.ObjWidth[objId] = obj.size
                objId = objId + 1

            if type(obj) is Coin and objId < 20:
                s.ObjID[objId] = 2
                delta_x = obj.pos[0] - v.state[0]
                delta_y = obj.pos[1] - v.state[1]
                s.ObjX[objId] = delta_x * cos(v.state[2]) + delta_y * sin(v.state[2])
                s.ObjY[objId] = - delta_x * sin(v.state[2]) + delta_y * cos(v.state[2])
                s.ObjVel[objId] = 0
                s.ObjLen[objId] = obj.size
                s.ObjWidth[objId] = obj.size
                objId = objId + 1

            if type(obj) is SuggestedSpeedSignal:
                s.AdasisSpeedLimitValues[speedlimitId] = obj.vel
                s.AdasisSpeedLimitDist[speedlimitId] = obj.pos[0] - v.state[0]
                speedlimitId = speedlimitId + 1

        s.NrObjs = objId
        s.AdasisSpeedLimitNr = speedlimitId

        s.NrTrfLights = 0
        if trafficlight:
            s.NrTrfLights = 1
            s.TrfLightDist = trafficlight.pos[0] - v.state[0]
            s.TrfLightCurrState = trafficlight.state + 1  # 1 = Green, 2 = Yellow, 3 = Red, 0 = Flashing
            s.TrfLightFirstTimeToChange = trafficlight.time_phases[
                                              trafficlight.state] - trafficlight.time_past_switch
            s.TrfLightFirstNextState = divmod(trafficlight.state + 1, 3)[1] + 1
            s.TrfLightSecondTimeToChange = s.TrfLightFirstTimeToChange + trafficlight.time_phases[
                divmod(trafficlight.state + 1, 3)[1]]
            s.TrfLightSecondNextState = divmod(trafficlight.state + 2, 3)[1] + 1
            s.TrfLightThirdTimeToChange = s.TrfLightSecondTimeToChange + trafficlight.time_phases[
                divmod(trafficlight.state + 2, 3)[1]]

        # print("CS:" + str(s.TrfLightDist))
        # print("CS:" + str(s.TrfLightCurrState))
        # print("NS:(" + str(s.TrfLightFirstTimeToChange) + "," + str(s.TrfLightFirstNextState) + ")")
        # print("NNS:(" + str(s.TrfLightSecondTimeToChange) + "," + str(s.TrfLightSecondNextState) + ")")
        # print("NNNS:(" + str(s.TrfLightThirdTimeToChange) +")")

        c.client_agent_compute(self.scenario_msg_pointer, self.manoeuvre_msg_pointer)

        # print("ID = " + str(m.ID))
        # print("Version = " + str(m.Version))
        # print("CycleNumber = " + str(m.CycleNumber))
        # print("ECUupTime = " + str(m.ECUupTime))
        # print("Status = " + str(m.Status))
        # print("CycleNumber = " + str(m.CycleNumber))
        # print("RequestedAcc = " + str(m.RequestedAcc))

        #self.action = (0.01, 0.01)
        self.action = (m.RequestedAcc,m.RequestedSteerWhlAg)

    def terminate(self):
        World().loop = 0

        self.cycle_number += 1
        self.scenario_msg.CycleNumber = self.cycle_number
        self.scenario_msg.TimeStamp = ct.c_double(datetime.timestamp(datetime.now()))
        self.scenario_msg.ECUupTime = World().time;
        self.scenario_msg.Status = 1

        c.client_agent_compute(self.scenario_msg_pointer, self.manoeuvre_msg_pointer)
        c.client_agent_close()

    def get_action(self):
        return self.action
