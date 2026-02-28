# drone

**Drone Surveillance Simulation (MATLAB)**

MATLAB simulation of an autonomous drone performing systematic surveillance over an N × N park, searching for fires or missing persons. The drone follows a waypoint-based coverage strategy using a double integrator dynamic model and a PID controller for trajectory tracking.

System Model

The drone is modeled as a 2D double integrator. The system states are position (x, y) and velocity (vx, vy). The control inputs are accelerations in the x and y directions, and the outputs are the planar positions of the drone.

Control Strategy

Waypoint tracking is achieved using a PID controller. The implementation includes derivative filtering to reduce noise amplification, integral anti-windup protection, and acceleration and velocity saturation limits. These mechanisms ensure stable numerical behavior and physically realistic motion during the surveillance mission.

Mission Logic

At runtime, the user selects the mission type. Fire missions use a 5×5 km detection window with 4 km lane spacing, while missing person missions use a 3×3 km detection window with 2 km lane spacing. The drone performs a systematic zig-zag scanning pattern to guarantee full coverage of the park while respecting the selected detection constraints. A visitation matrix tracks explored cells.

Features

Deterministic simulation using a fixed random seed
Configurable park size
Cell visitation tracking
Waypoint-based navigation
Saturation handling for motion constraints

Requirements

MATLAB (tested in recent versions). No additional toolboxes required.

Running the Simulation

Open MATLAB and run the main script. When prompted, select the mission type (fire or missing person) and the number of each one to be searched.
