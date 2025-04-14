#!/bin/bash

# This script removes files from the root directory that have been moved to appropriate folders
# in the reorganized project structure

echo "Cleaning up duplicate files from the root directory..."

# Files that have been moved to src/models/
echo "Removing duplicate model files..."
rm -f actuator_models.slx
rm -f actuator_models.slxc
rm -f STEVE_Simulation.slx
rm -f STEVE_Simulation.slxc
rm -f STEVE_Simulation_faultInfo.xml

# Files that have been moved to src/scripts/
echo "Removing duplicate script files..."
rm -f initialize_parameters.m
rm -f ControlSystemDesign.m
rm -f GimbalThrustModel.m
rm -f STeVe_CG_CP.m
rm -f visualizeRocketSimulation.m
rm -f NozzleActuatorCalculations.mlx
rm -f rocket_trajectory_animation.m

# Files that have been moved to src/visualization/
echo "Removing duplicate visualization files..."
rm -f create_rocket_kml.py
rm -f rocketpy_animation.py

# Files that have been moved to data directories
echo "Removing duplicate data files..."
rm -f "STeVe V1 No Fins.xlsx"
rm -f STeVe-Contour.xlsx
rm -f lookup_data.mat
rm -f rocket_trajectory_100fps.csv
rm -f RocketSimData.mat

# Files that have been moved to docs/
echo "Removing duplicate documentation files..."
rm -f README_old.md
rm -f electron-rocket-lab-3d-model-3146cc6b1b.jpg
rm -f steve.png

# Files that have been moved to output/
echo "Removing duplicate output files..."
rm -f rocket_trajectory.gif
rm -f rocket_trajectory_static.kml

echo "Cleanup complete!"
