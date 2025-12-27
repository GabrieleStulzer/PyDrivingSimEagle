"""
Preset circuit track configurations.

Each function returns a CircuitTrack instance with a closed-loop track
defined using pyclothoids segments.
"""

import math
from typing import List, Tuple

from pyclothoids import Clothoid

from scenarios.circuit_track import CircuitTrack


def create_oval_track(straight_length: float = 80.0,
                      turn_radius: float = 20.0,
                      track_width: float = 5.0) -> CircuitTrack:
    """
    Create an oval track with two straights and two semicircular turns.

    Layout:
          ___________
         /           \\
        |             |
        |             |
         \\___________/

    The track starts at the bottom-left corner, heading right (+X direction).

    Args:
        straight_length: Length of each straight section in meters.
        turn_radius: Radius of the semicircular turns in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    segments = []

    # Starting point: bottom center-left, facing right
    x, y, theta = 0.0, 0.0, 0.0

    # Segment 1: Bottom straight (left to right)
    seg1 = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length)
    segments.append(seg1)
    x, y, theta = seg1.XEnd, seg1.YEnd, seg1.ThetaEnd

    # Segment 2: Right turn (semicircle, curvature = 1/radius, positive = left turn)
    # For a semicircle: arc_length = pi * radius
    kappa = 1.0 / turn_radius
    arc_length = math.pi * turn_radius
    seg2 = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg2)
    x, y, theta = seg2.XEnd, seg2.YEnd, seg2.ThetaEnd

    # Segment 3: Top straight (right to left)
    seg3 = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length)
    segments.append(seg3)
    x, y, theta = seg3.XEnd, seg3.YEnd, seg3.ThetaEnd

    # Segment 4: Left turn (semicircle)
    seg4 = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg4)

    return CircuitTrack(segments, track_width)


def create_figure_eight_track(size: float = 50.0,
                              track_width: float = 5.0) -> CircuitTrack:
    """
    Create a figure-8 track with two loops.

    Layout:
          _____
         /     \\
        |   o   |
         \\ ___ /
         / ___ \\
        |   o   |
         \\_____/

    The track starts at the center intersection, heading right.
    First loop goes counter-clockwise (left turns), second loop clockwise (right turns).

    Args:
        size: Overall size parameter - diameter of each loop in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    segments = []
    radius = size / 2.0
    kappa = 1.0 / radius
    full_circle_length = 2 * math.pi * radius

    # Start at center intersection, facing right
    x, y, theta = 0.0, 0.0, 0.0

    # Top loop (counter-clockwise, positive curvature = left turns)
    seg1 = Clothoid.StandardParams(x, y, theta, kappa, 0.0, full_circle_length)
    segments.append(seg1)
    x, y, theta = seg1.XEnd, seg1.YEnd, seg1.ThetaEnd

    # Bottom loop (clockwise, negative curvature = right turns)
    seg2 = Clothoid.StandardParams(x, y, theta, -kappa, 0.0, full_circle_length)
    segments.append(seg2)

    return CircuitTrack(segments, track_width)


def create_racetrack(scale: float = 1.0,
                     track_width: float = 5.0) -> CircuitTrack:
    """
    Create a more complex racetrack with varied corners.

    Features:
    - Long back straight
    - Hairpin turn
    - S-curves (chicane)
    - Sweeping fast corner

    Args:
        scale: Scale factor for track size (1.0 = default size).
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    # Define waypoints with position and heading
    # The track will connect these points using G1 Hermite interpolation
    waypoints = [
        # (x, y, theta) - position and heading at each waypoint
        (0, 0, 0),                                          # Start/finish line
        (80 * scale, 0, 0),                                 # End of main straight
        (100 * scale, 25 * scale, math.pi / 2),             # Turn 1 - sweeping right
        (90 * scale, 60 * scale, math.pi * 0.9),            # Hairpin entry
        (70 * scale, 65 * scale, math.pi),                  # Hairpin apex
        (50 * scale, 55 * scale, -math.pi * 0.7),           # Hairpin exit
        (40 * scale, 35 * scale, -math.pi / 2),             # Chicane entry
        (50 * scale, 15 * scale, -math.pi / 6),             # Chicane mid
        (30 * scale, 5 * scale, math.pi),                   # Chicane exit
        (10 * scale, 10 * scale, math.pi * 0.8),            # Final corner entry
    ]

    segments = []

    # Connect waypoints with G1 Hermite interpolation
    for i in range(len(waypoints)):
        wp0 = waypoints[i]
        wp1 = waypoints[(i + 1) % len(waypoints)]

        seg = Clothoid.G1Hermite(
            wp0[0], wp0[1], wp0[2],  # Start: x, y, theta
            wp1[0], wp1[1], wp1[2]   # End: x, y, theta
        )
        segments.append(seg)

    return CircuitTrack(segments, track_width)


def create_simple_circuit(radius: float = 30.0,
                          track_width: float = 5.0) -> CircuitTrack:
    """
    Create a simple circular track.

    This is the simplest possible closed circuit - a single circle.
    Useful for testing and basic controller tuning.

    Args:
        radius: Radius of the circle in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    # Single circular segment
    kappa = 1.0 / radius
    circumference = 2 * math.pi * radius

    # Start at bottom of circle, heading right (tangent to circle)
    x, y, theta = 0.0, -radius, 0.0

    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, circumference)

    return CircuitTrack([seg], track_width)


def create_sinusoidal_track(amplitude: float = 3.0,
                            wavelength: float = 40.0,
                            length: float = 200.0,
                            track_width: float = 4.0) -> CircuitTrack:
    """
    Create a sinusoidal track using clothoid segments.

    The track follows a sinusoidal path: y = amplitude * sin(2*pi*x / wavelength)
    from x=0 to x=length, then loops back.

    Args:
        amplitude: Amplitude of the sinusoid in meters.
        wavelength: Wavelength of the sinusoid in meters.
        length: Total length of track in x-direction in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    # Generate waypoints along the sinusoidal path
    freq = 2 * math.pi / wavelength
    num_waypoints = int(length / (wavelength / 8)) + 1  # ~8 waypoints per wavelength
    spacing = length / num_waypoints

    waypoints = []

    # Forward path along sinusoid
    for i in range(num_waypoints + 1):
        x = i * spacing
        y = amplitude * math.sin(freq * x)
        # Heading is tangent to sinusoid: dy/dx = amplitude * freq * cos(freq * x)
        dy_dx = amplitude * freq * math.cos(freq * x)
        theta = math.atan2(dy_dx, 1.0)
        waypoints.append((x, y, theta))

    # Return path (offset to the side, going back)
    return_offset = amplitude * 3 + track_width  # Offset to avoid overlapping
    for i in range(num_waypoints, -1, -1):
        x = i * spacing
        y = amplitude * math.sin(freq * x) + return_offset
        dy_dx = amplitude * freq * math.cos(freq * x)
        theta = math.atan2(-dy_dx, -1.0)  # Reverse direction
        waypoints.append((x, y, theta))

    # Create clothoid segments connecting waypoints
    segments = []
    for i in range(len(waypoints)):
        wp0 = waypoints[i]
        wp1 = waypoints[(i + 1) % len(waypoints)]

        seg = Clothoid.G1Hermite(
            wp0[0], wp0[1], wp0[2],
            wp1[0], wp1[1], wp1[2]
        )
        segments.append(seg)

    return CircuitTrack(segments, track_width)


def create_halfcircle_track(radius: float = 30.0,
                            straight_length: float = 20.0,
                            track_width: float = 5.0) -> CircuitTrack:
    """
    Create a half-circle track with straight entry and exit.

    This is a simple open-loop style track useful for basic controller testing.
    The vehicle drives straight, goes through a 180-degree turn, and returns.

    Layout:
                 _______
                /       \\
               |         |
        -------|         |-------
        entry            exit
               |         |
                \\_______/

    The track starts with a straight, then a semicircle, then returns
    on another straight, completing the loop with another semicircle.

    Args:
        radius: Radius of the semicircular turns in meters.
        straight_length: Length of the straight sections in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    segments = []

    # Starting point: facing right (+X direction)
    x, y, theta = 0.0, 0.0, 0.0

    # Segment 1: Entry straight
    seg1 = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length)
    segments.append(seg1)
    x, y, theta = seg1.XEnd, seg1.YEnd, seg1.ThetaEnd

    # Segment 2: First semicircle (180-degree left turn)
    kappa = 1.0 / radius
    arc_length = math.pi * radius
    seg2 = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg2)
    x, y, theta = seg2.XEnd, seg2.YEnd, seg2.ThetaEnd

    # Segment 3: Return straight (going back, now facing left)
    seg3 = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length)
    segments.append(seg3)
    x, y, theta = seg3.XEnd, seg3.YEnd, seg3.ThetaEnd

    # Segment 4: Second semicircle (completes the loop)
    seg4 = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg4)

    return CircuitTrack(segments, track_width)


def create_custom_track(waypoints: List[Tuple[float, float, float]],
                        track_width: float = 5.0) -> CircuitTrack:
    """
    Create a custom track from a list of waypoints.

    Uses G1 Hermite interpolation to create smooth clothoid segments
    connecting each waypoint. The track automatically closes from the
    last waypoint back to the first.

    Args:
        waypoints: List of (x, y, theta) tuples defining track control points.
                   Each tuple specifies:
                   - x, y: Position in meters
                   - theta: Heading/tangent angle in radians
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.

    Example:
        waypoints = [
            (0, 0, 0),                    # Start facing right
            (50, 0, 0),                   # Straight section
            (70, 20, math.pi/2),          # Turn up
            (50, 40, math.pi),            # Turn left
            (0, 40, math.pi),             # Straight back
            (-20, 20, -math.pi/2),        # Turn down to close
        ]
        track = create_custom_track(waypoints, track_width=5.0)
    """
    if len(waypoints) < 2:
        raise ValueError("At least 2 waypoints are required")

    segments = []

    for i in range(len(waypoints)):
        wp0 = waypoints[i]
        wp1 = waypoints[(i + 1) % len(waypoints)]

        seg = Clothoid.G1Hermite(
            wp0[0], wp0[1], wp0[2],
            wp1[0], wp1[1], wp1[2]
        )
        segments.append(seg)

    return CircuitTrack(segments, track_width)
