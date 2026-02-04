import numpy as np

from pydrivingsim import TrafficLight, Target, TrafficCone, SuggestedSpeedSignal, GraphicObject, Vehicle, Agent, Coin, World

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


class ExcitationDriver():
    def __init__(self,
                 seed=1,
                 hold_time=0.2,
                 pedal_range=(-0.6, 0.8),
                 steer_range=(-0.35, 0.35),
                 warmup_time=1.0,
                 speed_range=(0.0, 12.0),
                 accel_long_range=(-2.0, 2.0),
                 accel_lat_range=(-1.5, 1.5),
                 episode_duration=5.0):
        self.vehicle = Vehicle()
        self.vehicle.set_screen_here()
        self.vehicle.set_pos_ang((0, -1, 0))

        self._rng = np.random.default_rng(seed)
        self._hold_time = hold_time
        self._pedal_range = pedal_range
        self._steer_range = steer_range
        self._warmup_time = warmup_time
        self._speed_range = speed_range
        self._accel_long_range = accel_long_range
        self._accel_lat_range = accel_lat_range
        self._episode_duration = episode_duration
        self._next_switch_time = 0.0
        self._action = (0.0, 0.0)
        self.episode_id = 0
        self._episode_start_time = 0.0
        self._start_episode(0.0)

    def _sample_action(self):
        pedal = self._rng.uniform(self._pedal_range[0], self._pedal_range[1])
        steer = self._rng.uniform(self._steer_range[0], self._steer_range[1])
        self._action = (pedal, steer)

    def _randomize_state(self):
        self.vehicle.reset()
        self.vehicle.set_pos_ang((0, -1, 0))
        self.vehicle.set_screen_here()
        u = self._rng.uniform(self._speed_range[0], self._speed_range[1])
        self.vehicle.state[3] = u
        self.vehicle.state[4] = 0.0
        steer = self._rng.uniform(self._steer_range[0], self._steer_range[1])
        self.vehicle.state[10] = steer
        self.vehicle.dX[3] = self._rng.uniform(self._accel_long_range[0], self._accel_long_range[1])
        self.vehicle.dX[4] = self._rng.uniform(self._accel_lat_range[0], self._accel_lat_range[1])
        wheel_r = self.vehicle.vehicle.front_wheel.R
        omega = u / wheel_r if wheel_r > 0 else 0.0
        self.vehicle.state[11] = omega
        self.vehicle.state[12] = omega
        self.vehicle.state[13] = omega
        self.vehicle.state[14] = omega

    def _start_episode(self, now):
        self._episode_start_time = now
        self._next_switch_time = now
        self._action = (0.0, 0.0)
        self._randomize_state()

    @property
    def episode_time(self):
        return World().time - self._episode_start_time

    def update(self):
        now = World().time
        if now - self._episode_start_time >= self._episode_duration:
            self.episode_id += 1
            self._start_episode(now)

        if self.episode_time < self._warmup_time:
            self._action = (0.0, 0.0)
        elif now >= self._next_switch_time:
            self._sample_action()
            self._next_switch_time = now + self._hold_time

        self.vehicle.set_screen_here()
        self.vehicle.control([self._action[0], self._action[1]])

    def terminate(self):
        pass

class SinusoidalAccelDriver():
    def __init__(self,
                 steer_amplitude=0.2,
                 steer_frequency=0.2,
                 steer_phase=0.0,
                 steer_ramp_time=2.0,
                 accel_mps2=0.5,
                 pedal_gain=0.3,
                 pedal_range=(-0.6, 0.8),
                 steer_limit=0.5,
                 start_speed=4):
        self.vehicle = Vehicle()
        self.vehicle.set_screen_here()
        self.vehicle.set_pos_ang((0, -1, 0))

        self._steer_amplitude = steer_amplitude
        self._steer_frequency = steer_frequency
        self._steer_phase = steer_phase
        self._steer_ramp_time = steer_ramp_time
        self._accel_mps2 = accel_mps2
        self._pedal_gain = pedal_gain
        self._pedal_range = pedal_range
        self._steer_limit = steer_limit
        self._start_speed = start_speed
        self._start_time = World().time
        self._reset_state()

    def _reset_state(self):
        self.vehicle.reset()
        self.vehicle.set_pos_ang((0, -1, 0))
        self.vehicle.set_screen_here()
        self.vehicle.state[3] = self._start_speed
        self.vehicle.state[4] = 0.0
        wheel_r = self.vehicle.vehicle.front_wheel.R
        omega = self._start_speed / wheel_r if wheel_r > 0 else 0.0
        self.vehicle.state[11] = omega
        self.vehicle.state[12] = omega
        self.vehicle.state[13] = omega
        self.vehicle.state[14] = omega

    @property
    def episode_time(self):
        return World().time - self._start_time

    def _compute_steer(self, t):
        if self._steer_ramp_time > 0.0:
            ramp = min(t / self._steer_ramp_time, 1.0)
        else:
            ramp = 1.0
        amplitude = ramp * self._steer_amplitude
        steer = amplitude * np.sin(2.0 * np.pi * self._steer_frequency * t + self._steer_phase)
        return float(np.clip(steer, -self._steer_limit, self._steer_limit))

    def _compute_pedal(self, t):
        target_speed = self._start_speed + self._accel_mps2 * t
        speed_error = target_speed - self.vehicle.state[3]
        pedal = self._pedal_gain * speed_error
        return float(np.clip(pedal, self._pedal_range[0], self._pedal_range[1]))

    def update(self):
        t = self.episode_time
        pedal = self._compute_pedal(t)
        steer = self._compute_steer(t)
        self.vehicle.set_screen_here()
        self.vehicle.control([pedal, steer])

    def terminate(self):
        pass

class SinusoidalAccelerationFixedSteerDriver():
    def __init__(self,
                 accel_amplitude=1.0,
                 accel_frequency=0.2,
                 accel_phase=0.0,
                 accel_ramp_time=2.0,
                 steer_value=0.1,
                 pedal_gain=0.3,
                 pedal_range=(-0.6, 0.8),
                 steer_limit=0.5,
                 start_speed=0.0):
        self.vehicle = Vehicle()
        self.vehicle.set_screen_here()
        self.vehicle.set_pos_ang((0, -1, 0))

        self._accel_amplitude = accel_amplitude
        self._accel_frequency = accel_frequency
        self._accel_phase = accel_phase
        self._accel_ramp_time = accel_ramp_time
        self._steer_value = steer_value
        self._pedal_gain = pedal_gain
        self._pedal_range = pedal_range
        self._steer_limit = steer_limit
        self._start_speed = start_speed
        self._start_time = World().time
        self._target_speed = start_speed
        self._last_time = 0.0
        self._reset_state()

    def _reset_state(self):
        self.vehicle.reset()
        self.vehicle.set_pos_ang((0, -1, 0))
        self.vehicle.set_screen_here()
        self.vehicle.state[3] = self._start_speed
        self.vehicle.state[4] = 0.0
        self._target_speed = self._start_speed
        self._last_time = 0.0
        wheel_r = self.vehicle.vehicle.front_wheel.R
        omega = self._start_speed / wheel_r if wheel_r > 0 else 0.0
        self.vehicle.state[11] = omega
        self.vehicle.state[12] = omega
        self.vehicle.state[13] = omega
        self.vehicle.state[14] = omega

    @property
    def episode_time(self):
        return World().time - self._start_time

    def _compute_target_speed(self, t):
        if self._accel_ramp_time > 0.0:
            ramp = min(t / self._accel_ramp_time, 1.0)
        else:
            ramp = 1.0
        omega = 2.0 * np.pi * self._accel_frequency
        accel = ramp * self._accel_amplitude * np.sin(omega * t + self._accel_phase)
        dt = max(0.0, t - self._last_time)
        self._last_time = t
        self._target_speed += accel * dt
        return self._target_speed

    def _compute_pedal(self, t):
        target_speed = self._compute_target_speed(t)
        speed_error = target_speed - self.vehicle.state[3]
        pedal = self._pedal_gain * speed_error
        return float(np.clip(pedal, self._pedal_range[0], self._pedal_range[1]))

    def update(self):
        t = self.episode_time
        pedal = self._compute_pedal(t)
        steer = float(np.clip(self._steer_value, -self._steer_limit, self._steer_limit))
        self.vehicle.set_screen_here()
        self.vehicle.control([pedal, steer])

    def terminate(self):
        pass

def SinusoidalTrack(amplitude=3.0, wavelength=40.0, length=200.0, track_width=4.0,
                    cone_spacing=5.0, max_speed=10.0):
    """
    Create a sinusoidal track scenario using CircuitTrackScenario.

    This is a convenience function that creates a sinusoidal track
    using the clothoid-based CircuitTrackScenario infrastructure.

    Args:
        amplitude: Amplitude of sinusoid in meters.
        wavelength: Wavelength of sinusoid in meters.
        length: Length of track in x-direction in meters.
        track_width: Full width of track in meters.
        cone_spacing: Distance between cones along track in meters.
        max_speed: Maximum target speed for the controller.

    Returns:
        CircuitTrackScenario instance.
    """
    from scenarios.circuit_track import CircuitTrackScenario
    return CircuitTrackScenario.sinusoidal(
        amplitude=amplitude,
        wavelength=wavelength,
        length=length,
        track_width=track_width,
        cone_spacing=cone_spacing,
        max_speed=max_speed
    )


def HalfCircleTrack(radius=30.0, straight_length=20.0, track_width=5.0,
                    cone_spacing=3.0):
    """
    Create a half-circle track scenario for basic controller testing.

    Simple track with straight entry, 180-degree turn, straight return,
    and another 180-degree turn to close the loop.

    Args:
        radius: Radius of the semicircular turns in meters.
        straight_length: Length of straight sections in meters.
        track_width: Full width of track in meters.
        cone_spacing: Distance between cones along track in meters.

    Returns:
        CircuitTrackScenario instance.
    """
    from scenarios.circuit_track import CircuitTrackScenario
    return CircuitTrackScenario.halfcircle(
        radius=radius,
        straight_length=straight_length,
        track_width=track_width,
        cone_spacing=cone_spacing
    )
