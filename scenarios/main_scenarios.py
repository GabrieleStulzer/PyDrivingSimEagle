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
