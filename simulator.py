# Authors : Gastone Pietro Rosati Papini
# Date    : 09/08/2022
# License : MIT
import signal

from pydrivingsim import World
from scenarios import BasicSpeedLimit, BasicTrafficLight, OnlyVehicle, AutonomousVehicle, GetTheCoins, SinusoidalTrack, HalfCircleTrack, CircuitTrackScenario

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
    av = SinusoidalTrack()
    # av = HalfCircleTrack(radius=30.0, straight_length=20.0, track_width=5.0, cone_spacing=3.0)
    # av = CircuitTrackScenario.oval(straight_length=80, turn_radius=25, track_width=5.0, cone_spacing=3.0, max_speed=12.0)
    #BasicTrafficLight()
    # Enable this to test the coins
    #GetTheCoins()
    # Enable this to test the speed limit
    #BasicSpeedLimit()

    killer = GracefulKiller()
    while not killer.kill_now and World().loop:
        av.update()
        World().update()

    av.terminate()
    World().exit()

main()
