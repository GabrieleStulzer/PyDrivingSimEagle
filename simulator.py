# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT
import signal

from pydrivingsim import World
from pydrivingsim.data_logger import DataLogger
from scenarios import BasicSpeedLimit, BasicTrafficLight, OnlyVehicle, AutonomousVehicle, GetTheCoins, SinusoidalTrack, HalfCircleTrack, CircuitTrackScenario, ExcitationDriver, SinusoidalAccelDriver, SinusoidalAccelerationFixedSteerDriver

class GracefulKiller:
  kill_now = False
  def __init__(self):
    signal.signal(signal.SIGINT, self.exit_gracefully)
    signal.signal(signal.SIGTERM, self.exit_gracefully)

  def exit_gracefully(self, *args):
    self.kill_now = True

def main():
    # Enable this to test only single vehicle
    # av = OnlyVehicle()
    # av = AutonomousVehicle()
    # av = SinusoidalTrack()
    # av = ExcitationDriver()
    av = SinusoidalAccelDriver(steer_amplitude=0.8, steer_frequency=0.2, accel_mps2=0.22, pedal_gain=0.1)
    # av = SinusoidalAccelerationFixedSteerDriver()
    # av = HalfCircleTrack(radius=30.0, straight_length=20.0, track_width=5.0, cone_spacing=3.0)
    # av = CircuitTrackScenario.oval(straight_length=80, turn_radius=25, track_width=5.0, cone_spacing=3.0, max_speed=12.0)
    #BasicTrafficLight()
    # Enable this to test the coins
    #GetTheCoins()
    # Enable this to test the speed limit
    #BasicSpeedLimit()

    killer = GracefulKiller()
    logger = DataLogger(sample_dt=0.01, separate_episodes=True)
    vehicle = getattr(av, "vehicle", None)
    last_episode_id = getattr(av, "episode_id", 0)
    logger.start_episode(last_episode_id, start_time_s=World().time)
    try:
        while not killer.kill_now and World().loop:
            av.update()
            World().update()
            if vehicle is not None:
                episode_id = getattr(av, "episode_id", 0)
                if episode_id != last_episode_id:
                    logger.start_episode(episode_id, start_time_s=World().time)
                    last_episode_id = episode_id
                episode_time = getattr(av, "episode_time", None)
                logger.log(World().time, vehicle.state, vehicle.dX, vehicle.action, episode_id=episode_id, episode_time=episode_time)
    finally:
        logger.close()

    av.terminate()
    World().exit()

main()
