# The basic scenarios
from scenarios.main_scenarios import AutonomousVehicle, OnlyVehicle, BasicSpeedLimit, BasicTrafficLight, GetTheCoins, SinusoidalTrack, HalfCircleTrack

# Circuit track scenarios
from scenarios.circuit_track import CircuitTrack, CircuitTrackController, CircuitTrackScenario
from scenarios.circuit_presets import (
    create_oval_track,
    create_figure_eight_track,
    create_racetrack,
    create_simple_circuit,
    create_sinusoidal_track,
    create_custom_track,
)
