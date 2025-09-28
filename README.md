# STEVE Rocket Model Simulation

A comprehensive MATLAB-based rocket simulation environment for modeling, simulating, and visualizing rocket trajectories with multi-platform visualization capabilities.


## Project Structure

The project is organized into the following folders:

```
.
├── MATLAB_Toolbox/            # MATLAB toolbox functions and control utilities
├── simulation_3dof/           # 3DOF simulation scripts, models, and setup
│   ├── scripts/                   # Main scripts and setup for 3DOF
│   ├── data/                      # Input/output data for 3DOF
│   └── Visualization/             # Visualization assets for 3DOF
├── simulation_6dof/           # 6DOF simulation scripts, models, and setup
│   ├── scripts/                   # Main scripts and setup for 6DOF
│   ├── data/                      # Input/output data for 6DOF
│   └── Visualization/             # Visualization assets for 6DOF
├── docs/                      # Documentation and images
│   └── images/
├── README.md                  # This file
├── .gitignore
└── (other files/folders as the project evolves)
```

## Overview

The STEVE Rocket Model Simulation is a high-fidelity simulation framework designed for engineering analysis of rocket flight dynamics. It includes sophisticated models for mass properties, aerodynamics, control systems, and environmental factors to provide accurate predictions of rocket behavior throughout all phases of flight.

## Key Features

### Physics-Based Simulation

- **6DOF Flight Dynamics**: Complete 3D rigid body dynamics with time-varying mass properties
- **Accurate Aerodynamics**: Mach and angle-of-attack dependent aerodynamic coefficients (CFD-supported)
- **Variable Mass Model**: Time-varying mass, center of gravity, and moments of inertia
- **Thrust Vector Control**: Realistic nozzle actuation with rate limiting and dynamic response
- **Wind Effects**: Configurable wind profiles including wind shear

### Control Architecture

#### Inner Loop Stability + Feedforward
1. **Inner loop pitch rate controller (PD) for disturbance rejection + pitch rate tracking**
   - Gain scheduled via:  
     - Time: CG position, moment of inertia  
     - Altitude: Thrust force
2. **Aerodynamic moment feedforward controller**
   - Estimates aerodynamic moment on the vehicle
   - Estimated parameters:  
     - Angle of attack  
     - Airspeed  
     - Center of Pressure (CP)  
     - Aerodynamic pitching moment coefficient (CFD)  
   - Adaptive feedforward (tunes weights by monitoring PD controller effort)
3. **Mid loop pitch angle controller**
4. **Outer loop trajectory tracking controllers**

### Comprehensive Analysis Tools

- **Flight Metrics**: Detailed calculation of key performance indicators including:
  - Maximum altitude and speed
  - Burnout conditions (altitude, speed, time)
  - Maximum acceleration and angle of attack
  - Flight time and trajectory statistics

- **Visualization Suite**: In-depth 2D/3D analysis:
  - Trajectory plotting
  - Velocity components (Earth/Body frame)
  - Attitude/angular rates
  - Force and moment analysis
  - Wind visualization

### Multi-Platform Visualization

- **MATLAB Plotting**: 2D/3D data analysis tools
- **Python 3D Animation**: Real-time visualization with PyVista and Qt
- **Blender Integration**: High-quality 3D rendering of flight data
- **Google Earth Export**: KML trajectory overlays for geospatial results

## Usage

### Basic Simulation


### 3DOF Simulation
1. Run `simulation_3dof/scripts/run_simulation.m` in MATLAB to execute the 3DOF simulation:
  - Initializes parameters, configures control, runs model, visualizes results, exports for external platforms

### 6DOF Simulation
1. Run `simulation_6dof/scripts/run_simulation.m` in MATLAB to execute the 6DOF simulation:
  - Uses `initialize_sim_params_6dof.m` and other updated setup scripts
  - Provides full 6DOF flight dynamics and analysis

_Alternatively, run each step individually in the relevant scripts for advanced workflows._

### Extended Visualization


#### 3D Animation with Python
1. Run `simulation_3dof/Visualization/visualizeRocketSimulation.m` or `simulation_6dof/Visualization/visualizeRocketSimulation.m` in MATLAB to generate data
2. Run `Visualization/Python/rocketpy_animation.py` for 3D animation

#### Blender Animation
1. Install Blender (https://www.blender.org/)
2. Open `Visualization/Blender/RocketAnimation.blend`
3. Run `blender_animation.py` inside Blender

#### Google Earth Visualization
1. Run `Visualization/Python/create_rocket_kml.py` to convert trajectory data to KML
2. Open `Visualization/Generated_Media/rocket_trajectory_static.kml` in Google Earth

## Requirements


### MATLAB Requirements
- **MATLAB R2025a** or newer
- Simulink
- Control System Toolbox

### Python Requirements
- Python 3.8+
- PyVista, PyVistaQt
- NumPy, Pandas
- PyQt5 or PyQt6

### Optional Tools
- Blender 3.0+ for 3D rendering
- Google Earth for KML visualization

## License

This project is available for educational and research purposes. Please reference appropriately if used in academic work or derivative projects.

---

*Developed as part of aerospace engineering research focusing on rocket flight dynamics and control systems.*
