# drone
Drone surveillance simulation (MATLAB)
MATLAB simulation of an autonomous drone performing systematic surveillance over an N × N park, searching for fires or missing persons. The drone follows a waypoint-based coverage strategy using a double integrator dynamic model and a PID controller for trajectory tracking.

# system model
The drone is modeled as a 2D double integrator. The system states are position (x, y) and velocity (vx, vy). The control inputs are accelerations in the x and y directions, and the outputs are the planar positions of the drone.

# control strategy
Waypoint tracking is achieved using a PID controller. The implementation includes derivative filtering, integral anti-windup protection, and acceleration and velocity saturation limits to ensure stable numerical behavior and physically realistic motion.

# mission logic
At runtime, the user selects the mission type. Fire missions use a 5×5 km detection window with 4 km lane spacing, while missing person missions use a 3×3 km detection window with 2 km lane spacing. The drone performs a systematic zig-zag scanning pattern to guarantee full coverage of the park while respecting the selected detection constraints. A visitation matrix tracks explored cells.

# features
Deterministic simulation using a fixed random seed
Configurable park size
Cell visitation tracking
Waypoint-based navigation
Saturation handling for motion constraints

# requirements
MATLAB (recent versions). No additional toolboxes required.

# running the simulation
Open MATLAB and run the main script. When prompted, select the mission type (fire or missing person) and the number of targets to be searched.
