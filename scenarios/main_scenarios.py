import math

from pydrivingsim import TrafficLight, Target, TrafficCone, SuggestedSpeedSignal, GraphicObject, Vehicle, Agent, Coin

class OnlyVehicle():
    def __init__(self):
        #Initialize the vehicle
        self.vehicle = Vehicle()
        self.vehicle.set_screen_here()
        self.vehicle.set_pos_ang((0, -1, 0))

        #Initialize target
        target = Target()
        target.set_pos((182, -1))
        target.set_object(self.vehicle)

    def update(self):
        self.vehicle.set_screen_here()
        self.vehicle.control([0.5, 0.0])

    def terminate(self):
        pass

class AutonomousVehicle():
    def __init__(self):
        # Initialize the vehicle
        self.vehicle = Vehicle()
        self.vehicle.set_screen_here()
        self.vehicle.set_pos_ang((0, -1, 0))

        #Initialize the agent
        self.agent = Agent(self.vehicle)

        #Initialize target
        target = Target()
        target.set_pos((182, -1))
        target.set_object(self.vehicle)

    def update(self):
        self.agent.compute()
        action = self.agent.get_action()

        self.vehicle.set_screen_here()
        self.vehicle.control([action[0], action[1]])

    def terminate(self):
        self.agent.terminate()


class BasicTrafficLight():
    def __init__(self):
        cone = TrafficCone()
        cone.set_pos((1.0,0))
        cone = TrafficCone()
        cone.set_pos((1.0,2))
        cone = TrafficCone()
        cone.set_pos((1.0,-2))

        trafficlight = TrafficLight()
        trafficlight.set_pos((160,-3))
        trafficlight.reset()

class GetTheCoins():
    def __init__(self):
        # Added point in zero to be able to start with a reference
        coin = Coin()
        coin.set_pos((0,-1))

        # Targets to avoid cones
        coin = Coin()
        coin.set_pos((10,-1))
        coin = Coin()
        coin.set_pos((35,1))
        coin = Coin()
        coin.set_pos((60,-1))
        coin = Coin()
        coin.set_pos((100,1))
        coin = Coin()
        coin.set_pos((130,-1))

        # Add two final points to stabilize the trajectory
        coin = Coin()
        coin.set_pos((160,-1))
        coin = Coin()
        coin.set_pos((182,-1))


class BasicSpeedLimit():
    def __init__(self):
        signal = SuggestedSpeedSignal(10)
        signal.set_pos((50, 4))
        bologna = GraphicObject("imgs/pictures/bologna.png", 35)
        bologna.set_pos((67,12))
        signal = SuggestedSpeedSignal(90)
        signal.set_pos((96, 4))
        super = GraphicObject("imgs/pictures/superstrada.png", 5)
        super.set_pos((100,6))


def _wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class SinusoidalTrackController():
    def __init__(self, vehicle, amplitude, wavelength, min_speed, max_speed):
        self.vehicle = vehicle
        self.amplitude = amplitude
        self.wavelength = wavelength
        self.freq = (2 * math.pi) / wavelength
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.k_lat = 2 
        self.k_heading = -1.5
        self.k_speed = 0.6
        self.speed_gain = 15.0

    def _reference(self, x):
        y_ref = self.amplitude * math.sin(self.freq * x)
        dy_dx = self.amplitude * self.freq * math.cos(self.freq * x)
        yaw_ref = math.atan2(dy_dx, 1.0)
        curvature = (self.amplitude * (self.freq ** 2) * math.sin(self.freq * x)) / pow(1 + dy_dx ** 2, 1.5)
        return y_ref, yaw_ref, abs(curvature)

    def compute_action(self):
        state, _ = self.vehicle.get_state()
        x_pos, y_pos, yaw = state[0], state[1], state[2]
        vel = state[3]

        y_ref, yaw_ref, curvature = self._reference(x_pos)
        lateral_error = y_ref - y_pos
        heading_error = _wrap_angle(yaw_ref - yaw)

        steer = self.k_lat * lateral_error + self.k_heading * heading_error
        steer = _clamp(steer, -0.6, 0.6)

        speed_ref = self.max_speed - self.speed_gain * curvature
        speed_ref = _clamp(speed_ref, self.min_speed, self.max_speed)
        accel = self.k_speed * (speed_ref - vel)
        accel = _clamp(accel, -3.0, 3.0)

        return [accel, steer]


class SinusoidalTrack():
    def __init__(self, amplitude=3.0, wavelength=40.0, length=200.0, lane_width=4.0, spacing=5.0,
                 min_speed=6.0, max_speed=10.0):
        self.vehicle = Vehicle()
        start_y = amplitude * math.sin(0.0)
        self.vehicle.set_pos_ang((0.0, start_y, 0.0))
        self.vehicle.set_screen_here()

        self.controller = SinusoidalTrackController(self.vehicle, amplitude, wavelength, min_speed, max_speed)
        self._build_track(amplitude, wavelength, lane_width, length, spacing)

        target = Target()
        final_y = amplitude * math.sin((2 * math.pi / wavelength) * length)
        target.set_pos((length + 5.0, final_y))
        target.set_object(self.vehicle)
        self.target = target

    def _build_track(self, amplitude, wavelength, lane_width, length, spacing):
        freq = (2 * math.pi) / wavelength
        x_pos = 0.0
        while x_pos <= length:
            center_y = amplitude * math.sin(freq * x_pos)
            for offset in (-lane_width / 2.0, lane_width / 2.0):
                cone = TrafficCone()
                cone.set_pos((x_pos, center_y + offset))

            coin = Coin()
            coin.set_pos((x_pos, center_y))
            x_pos += spacing

    def update(self):
        self.vehicle.set_screen_here()
        action = self.controller.compute_action()
        self.vehicle.control(action)

    def terminate(self):
        pass
