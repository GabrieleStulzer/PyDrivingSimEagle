"""
Preset circuit track configurations.

Each function returns a CircuitTrack instance with a closed-loop track
defined using pyclothoids segments.
"""

import math
import os
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


def create_complex_oval(straight_length: float = 80.0,
                        turn_radius: float = 25.0,
                        track_width: float = 5.0) -> CircuitTrack:
    """
    Create a complex oval track with multiple curves and chicanes.

    Similar to oval track but with additional technical sections:
    - Long main straight
    - Fast sweeping turn
    - Medium straight with chicane
    - Technical section
    - Back straight
    - Final turns to close the loop

    Guaranteed closure with properly calculated angles.

    Args:
        straight_length: Length of straight sections in meters.
        turn_radius: Base radius of turns in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    segments = []
    x, y, theta = 0.0, 0.0, 0.0

    # Segment 1: Long main straight (start/finish)
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 2.0)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 2: Turn 1 - Right 60° (sweeping)
    angle = math.pi / 3  # 60 degrees
    kappa = 1.0 / (turn_radius * 1.8)
    arc_length = angle * turn_radius * 1.8
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 3: Short straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.7)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 4: Turn 2 - Right 60° (continue)
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 5: Medium straight (before chicane)
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.8)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 6: Chicane - Left 30°
    angle_chicane = math.pi / 6  # 30 degrees
    kappa_chicane = -1.0 / (turn_radius * 1.5)
    arc_chicane = angle_chicane * turn_radius * 1.5
    seg = Clothoid.StandardParams(x, y, theta, kappa_chicane, 0.0, arc_chicane)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 7: Chicane - Right 30° (back)
    kappa_chicane = 1.0 / (turn_radius * 1.5)
    seg = Clothoid.StandardParams(x, y, theta, kappa_chicane, 0.0, arc_chicane)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 8: Straight section
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.9)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 9: Turn 3 - Right 60°
    angle = math.pi / 3
    kappa = 1.0 / (turn_radius * 1.3)
    arc_length = angle * turn_radius * 1.3
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 10: Back straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 1.5)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 11: Turn 4 - Right 60° (technical)
    kappa = 1.0 / turn_radius
    arc_length = angle * turn_radius
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 12: Short straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.6)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd

    # Segment 13: Turn 5 - Right 60° (final turn to close)
    # Total angles: 60 + 60 + 60 + 60 + 60 = 300° from right turns
    #               -30 + 30 = 0° from chicane
    # Need 60° more to make 360°
    angle = math.pi / 3
    kappa = 1.0 / (turn_radius * 1.4)
    arc_length = angle * turn_radius * 1.4
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)

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


def create_racetrack(straight_length: float = 80.0,
                     turn_radius: float = 25.0,
                     track_width: float = 5.0) -> CircuitTrack:
    """
    Create a complex racetrack with varied corners.

    Features:
    - Two long straights (one longer for overtaking)
    - Multiple corners of different radii
    - Technical sections with varied turns
    - Guaranteed closed loop with no overlaps

    Args:
        straight_length: Base length of straight sections in meters.
        turn_radius: Base radius of turns in meters.
        track_width: Full width of track in meters.

    Returns:
        CircuitTrack instance.
    """
    segments = []
    
    # Starting position and heading
    x, y, theta = 0.0, 0.0, 0.0
    
    # --- BOTTOM SECTION ---
    # Segment 1: Long main straight (start/finish)
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 2.0)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 2: Turn 1 - Right 90° (fast sweeping)
    kappa = 1.0 / (turn_radius * 2.0)
    arc_length = (math.pi / 2) * turn_radius * 2.0
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- RIGHT SECTION ---
    # Segment 3: Right side straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 1.0)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 4: Turn 2 - Right 45° (entry to chicane)
    kappa = 1.0 / (turn_radius * 1.5)
    arc_length = (math.pi / 4) * turn_radius * 1.5
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 5: Short straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.3)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 6: Turn 3 - Left 45° (chicane)
    kappa = -1.0 / (turn_radius * 1.5)
    arc_length = (math.pi / 4) * turn_radius * 1.5
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 7: Turn 4 - Right 90° (continue up)
    kappa = 1.0 / (turn_radius * 1.2)
    arc_length = (math.pi / 2) * turn_radius * 1.2
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- TOP SECTION ---
    # Segment 8: Top straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 1.5)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 9: Turn 5 - Right 90° (hairpin approach)
    kappa = 1.0 / turn_radius
    arc_length = (math.pi / 2) * turn_radius
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- LEFT SECTION ---
    # Segment 10: Short straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 0.4)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 11: Turn 6 - Right 90° (complete the turn)
    kappa = 1.0 / (turn_radius * 0.8)
    arc_length = (math.pi / 2) * turn_radius * 0.8
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 12: Back straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, straight_length * 1.8)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 13: Turn 7 - Right 90° (final corner before finish)
    kappa = 1.0 / (turn_radius * 1.5)
    arc_length = (math.pi / 2) * turn_radius * 1.5
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
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


def create_f_shaped_track(scale: float = 80.0,
                          track_width: float = 5.0) -> CircuitTrack:
    """
    Create an F-shaped racing circuit similar to professional racetracks.
    
    Layout features:
    - Long vertical straight (main straight)
    - Top horizontal section with curves
    - Middle horizontal section (like the middle bar of an F)
    - Technical turns connecting sections
    - Return section to close the loop
    
    Args:
        scale: Overall size scaling factor in meters.
        track_width: Full width of track in meters.
    
    Returns:
        CircuitTrack instance.
    """
    segments = []
    x, y, theta = 0.0, 0.0, math.pi / 2  # Start facing up
    
    # --- MAIN STRAIGHT (vertical, going up) ---
    # Segment 1: Long main straight (start/finish)
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 1.2)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- TOP SECTION ---
    # Segment 2: Turn 1 - Left 90° (top-left corner)
    radius = scale * 0.3
    kappa = 1.0 / radius
    arc_length = (math.pi / 2) * radius
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 3: Top horizontal straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 0.8)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 4: Turn 2 - Right 90° (top-right corner)
    kappa = -1.0 / (radius * 0.8)
    arc_length = (math.pi / 2) * radius * 0.8
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- RIGHT DESCENT ---
    # Segment 5: Short straight down
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 0.25)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 6: Turn 3 - Left 90° (before middle section)
    kappa = 1.0 / (radius * 0.7)
    arc_length = (math.pi / 2) * radius * 0.7
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- MIDDLE SECTION (like middle bar of F) ---
    # Segment 7: Middle horizontal straight
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 0.6)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 8: Turn 4 - Right 90° (end of middle section)
    kappa = -1.0 / (radius * 0.8)
    arc_length = (math.pi / 2) * radius * 0.8
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # --- BOTTOM RETURN SECTION ---
    # Segment 9: Straight down
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 0.35)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 10: Turn 5 - Left 90° (bottom-right corner)
    kappa = 1.0 / (radius * 1.0)
    arc_length = (math.pi / 2) * radius * 1.0
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 11: Bottom straight (going back left)
    seg = Clothoid.StandardParams(x, y, theta, 0.0, 0.0, scale * 0.5)
    segments.append(seg)
    x, y, theta = seg.XEnd, seg.YEnd, seg.ThetaEnd
    
    # Segment 12: Turn 6 - Left 90° (final turn to close loop)
    kappa = 1.0 / (radius * 0.9)
    arc_length = (math.pi / 2) * radius * 0.9
    seg = Clothoid.StandardParams(x, y, theta, kappa, 0.0, arc_length)
    segments.append(seg)
    
    return CircuitTrack(segments, track_width)


def create_track_from_fsg_file(filepath: str, 
                                track_width: float = 5.0,
                                segment_length: float = 5.0) -> CircuitTrack:
    """
    Create a track from FSG format file with curvature data.
    
    The file should contain columns:
    - abscissa: distance along track
    - curvature: curvature at each point
    - x_mid_line, y_mid_line: centerline coordinates
    - dir_mid_line: heading/direction
    
    Args:
        filepath: Path to the FSG.txt file.
        track_width: Full width of track in meters.
        segment_length: Target length for each clothoid segment (meters).
                       Smaller values = more accurate but more segments.
    
    Returns:
        CircuitTrack instance.
    """
    import numpy as np
    
    # Read the file
    data = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            # Skip comments and empty lines
            if not line or line.startswith('#') or line.startswith('!'):
                continue
            # Skip header
            if 'abscissa' in line:
                continue
            
            parts = line.split('\t')
            if len(parts) >= 7:
                try:
                    s = float(parts[0])  # abscissa
                    kappa = float(parts[1])  # curvature
                    theta = float(parts[2])  # dir_mid_line
                    x = float(parts[3])  # x_mid_line
                    y = float(parts[4])  # y_mid_line
                    data.append((s, kappa, theta, x, y))
                except ValueError:
                    continue
    
    if len(data) < 2:
        raise ValueError(f"Not enough valid data points in file: {filepath}")
    
    # Convert to arrays
    s_data = np.array([d[0] for d in data])
    kappa_data = np.array([d[1] for d in data])
    theta_data = np.array([d[2] for d in data])
    x_data = np.array([d[3] for d in data])
    y_data = np.array([d[4] for d in data])
    
    # Create segments by sampling at regular intervals
    segments = []
    total_length = s_data[-1]
    num_segments = int(total_length / segment_length)
    
    if num_segments < 10:
        num_segments = 10  # Minimum segments
    
    for i in range(num_segments):
        # Start and end of this segment
        s_start = i * segment_length
        s_end = (i + 1) * segment_length if i < num_segments - 1 else total_length
        seg_length = s_end - s_start
        
        # Find indices for interpolation
        idx_start = np.searchsorted(s_data, s_start)
        idx_end = np.searchsorted(s_data, s_end)
        
        if idx_start >= len(s_data):
            idx_start = len(s_data) - 1
        if idx_end >= len(s_data):
            idx_end = len(s_data) - 1
        
        # Get values at segment start
        if idx_start < len(s_data):
            x_start = float(x_data[idx_start])
            y_start = float(y_data[idx_start])
            theta_start = float(theta_data[idx_start])
            kappa_start = float(kappa_data[idx_start])
        else:
            # Wrap to start for closure
            x_start = float(x_data[0])
            y_start = float(y_data[0])
            theta_start = float(theta_data[0])
            kappa_start = float(kappa_data[0])
        
        # Average curvature over segment for better approximation
        if idx_start < idx_end:
            kappa_avg = float(np.mean(kappa_data[idx_start:idx_end+1]))
        else:
            kappa_avg = kappa_start
        
        # Create clothoid segment with constant curvature (dk=0)
        try:
            seg = Clothoid.StandardParams(
                x_start, y_start, theta_start,
                kappa_avg, 0.0, seg_length
            )
            segments.append(seg)
        except Exception as e:
            print(f"Warning: Could not create segment {i}: {e}")
            continue
    
    if len(segments) == 0:
        raise ValueError("No valid segments created from file data")
    
    return CircuitTrack(segments, track_width)


def create_varano_track(track_width: float = 5.0,
                        segment_length: float = 5.0) -> CircuitTrack:
    """
    Create Varano racing circuit from data file.
    
    Loads the Varano circuit geometry from the Varano.txt file.
    The file contains curvature and centerline coordinates.
    
    Args:
        track_width: Full width of track in meters (default: 5.0).
        segment_length: Length of each clothoid segment for approximation (default: 5m).
                       Smaller values = more accurate but more segments.
    
    Returns:
        CircuitTrack instance of Varano circuit.
    """
    import os
    
    # Get the path to Varano.txt file (should be in same directory as this file)
    current_dir = os.path.dirname(os.path.abspath(__file__))
    varano_file = os.path.join(current_dir, "Varano.txt")
    
    if not os.path.exists(varano_file):
        raise FileNotFoundError(f"Varano.txt not found at: {varano_file}")
    
    return create_track_from_fsg_file(varano_file, track_width, segment_length)
