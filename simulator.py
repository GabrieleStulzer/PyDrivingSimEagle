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
    # av = SinusoidalAccelDriver(steer_amplitude=0.6, steer_frequency=0.05, accel_mps2=0.0, pedal_gain=0.2, start_speed=5)
    # av = SinusoidalAccelerationFixedSteerDriver()
    # av = HalfCircleTrack(radius=30.0, straight_length=20.0, track_width=5.0, cone_spacing=3.0)
    # av = CircuitTrackScenario.racetrack(radius=30.0, straight_length=20.0, track_width=5.0, cone_spacing=3.0)
    # av = CircuitTrackScenario.f_shaped(scale=80.0, track_width=5.0, cone_spacing=3.0)
    
    # Load circuit from file - you can use any of these:
    av = CircuitTrackScenario.circuit("scenarios/FSG.txt", track_width=5.0, segment_length=0.02, cone_spacing=5.0)
    # av = CircuitTrackScenario.circuit("scenarios/FSG.txt", track_width=5.0, segment_length=2.0, cone_spacing=5.0)
    # av = CircuitTrackScenario.varano(track_width=5.0, segment_length=2.0, cone_spacing=5.0)  # Alternative for Varano
    
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
