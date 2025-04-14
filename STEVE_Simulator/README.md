# STEVE Rocket Model Simulation

A comprehensive MATLAB-based rocket simulation environment for modeling, simulating, and visualizing rocket trajectories with multi-platform visualization capabilities.

![STEVE Rocket](docs/images/steve.png)

## Project Structure

The project is organized into the following folders:

```
STEVE_Simulator/
├── src/                          # Source code
│   ├── models/                   # Main simulation models
│   ├── scripts/                  # MATLAB scripts
│   ├── visualization/            # Visualization scripts
│   │   ├── matlab/               # MATLAB visualization
│   │   ├── python/               # Python visualization
│   │   └── blender_animation/    # Blender animation
│   └── util/                     # Utility functions
│
├── data/                         # Data files
│   ├── parameters/               # Configuration data
│   ├── lookup/                   # Lookup tables
│   ├── results/                  # Simulation results
│   └── aero/                     # Aerodynamics data
│
├── docs/                         # Documentation
│   └── images/                   # Documentation images
│
├── output/                       # Output files
│   ├── animations/               # Generated animations
│   └── kml/                      # Generated KML files
│
└── rocket_env/                   # Python environment
```

## Overview

The STEVE Rocket Model Simulation is a high-fidelity simulation framework designed for engineering analysis of rocket flight dynamics. It includes sophisticated models for mass properties, aerodynamics, control systems, and environmental factors to provide accurate predictions of rocket behavior throughout all phases of flight.

## Key Features

### Physics-Based Simulation

- **6DOF Flight Dynamics**: Complete 3D rigid body dynamics with time-varying mass properties
- **Accurate Aerodynamics**: Mach and angle-of-attack dependent aerodynamic coefficients
- **Variable Mass Model**: Time-varying mass, center of gravity, and moments of inertia
- **Thrust Vector Control**: Realistic nozzle actuation with rate limiting and dynamic response
- **Wind Effects**: Configurable wind profiles including wind shear

### Control System Design

- **Cascaded Control System**: Inner-loop rate control and outer-loop attitude control
- **Adjustable Control Parameters**: Configurable gains for performance tuning
- **Multiple Actuator Models**: Support for 1st and 2nd-order actuator dynamics

### Comprehensive Analysis Tools

- **Flight Metrics**: Detailed calculation of key performance parameters:
  - Maximum altitude and speed
  - Burnout conditions (altitude, speed, time)
  - Maximum acceleration and angle of attack
  - Flight time and trajectory statistics

- **Visualization Suite**: Multi-tabbed visualization interface for in-depth analysis:
  - 3D and 2D trajectory plotting
  - Velocity components (Earth-frame and Body-frame)
  - Attitude and angular rates
  - Force and moment analysis
  - Wind effects visualization

### Multi-Platform Visualization

- **MATLAB Plotting**: Comprehensive 2D plots for detailed data analysis
- **Python 3D Animation**: Real-time trajectory visualization using PyVista and Qt
- **Blender Integration**: High-quality 3D rendering using exported simulation data
- **Google Earth Export**: KML trajectory export for geospatial visualization

## Usage

### Basic Simulation

1. Simply run `src/scripts/runRocketSimulation.m` to execute the complete simulation workflow:
   - This script initializes parameters, configures the control system, runs the simulation model, visualizes results, and exports data for external visualization tools
   
   Alternatively, you can run each step individually:
   - Run `src/scripts/initialize_parameters.m` to set up simulation parameters
   - Run `src/scripts/ControlSystemDesign.m` to configure the control system
   - Run `src/models/STEVE_Simulation.slx` to execute the simulation

### Extended Visualization

#### 3D Animation with Python

The simulation includes a Python-based real-time 3D animation system:

1. Run `src/visualization/matlab/visualizeRocketSimulation.m` to generate trajectory data
2. Execute `src/visualization/python/rocketpy_animation.py` to view the 3D animation

#### Blender Animation

For high-quality rendering:

1. Install Blender (https://www.blender.org/)
2. Open the Blender file `src/visualization/blender_animation/RocketAnimation.blend`
3. Run the `src/visualization/blender_animation/blender_animation.py` script within Blender

#### Google Earth Visualization

1. Run `src/visualization/python/create_rocket_kml.py` to convert trajectory data to KML format
2. Open the generated `output/kml/rocket_trajectory_static.kml` file in Google Earth

## Requirements

### MATLAB Requirements
- MATLAB R2020b or newer
- Simulink
- Control System Toolbox

### Python Requirements
- Python 3.8+
- PyVista and PyVistaQt
- NumPy, Pandas
- PyQt5 or PyQt6

### Optional Tools
- Blender 3.0+ for 3D rendering
- Google Earth for KML visualization

## License

This project is available for educational and research purposes. Please reference appropriately if used in academic work or derivative projects.

---

*Developed as part of aerospace engineering research focusing on rocket flight dynamics and control systems.*
