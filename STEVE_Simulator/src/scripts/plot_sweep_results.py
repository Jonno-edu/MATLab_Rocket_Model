import os
import glob
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
import warnings
from scipy.interpolate import interp1d, RegularGridInterpolator # Added RegularGridInterpolator
from scipy.optimize import minimize_scalar, minimize # Added minimize
from matplotlib.collections import LineCollection # Added for Plot 3
from matplotlib.colors import Normalize # Added for Plot 3
import matplotlib.cm as cm # Added for Plot 3
from matplotlib.lines import Line2D
import h5py # Add h5py import

# Close any existing Matplotlib figures first
plt.close('all')

# --- Configuration ---
save_plots = False  # Set to True to save plots, False to only display them
results_dir_relative = '../../data/results' # Relative path from script location
plt.style.use('seaborn-v0_8-darkgrid') # Use a visually appealing style

# --- Setup Paths ---
print("Setting up paths...")
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
results_dir = os.path.abspath(os.path.join(script_dir, results_dir_relative))
print(f"Script directory: {script_dir}")
print(f"Project root: {project_root}")
print(f"Results directory: {results_dir}")

# --- Load Results File ---
# Use the fixed filename directly
results_filename = 'tilt_pitch_rate_sweep_results.mat' # Updated filename
results_file_path = os.path.join(results_dir, results_filename)

if not os.path.exists(results_file_path):
    raise FileNotFoundError(f"Results file not found at: {results_file_path}")

print(f"Loading results from: {results_file_path}")
try:
    # Use h5py to load v7.3 mat files
    with h5py.File(results_file_path, 'r') as f:
        # Helper function to recursively load data from HDF5 file
        def load_hdf5_group(group):
            data = {}
            for k, v in group.items():
                if isinstance(v, h5py.Dataset):
                    # Load dataset value
                    value = v[()] # Use [()] to read scalar/array data
                    # Handle potential byte strings (common for strings from MATLAB)
                    if isinstance(value, bytes):
                        value = value.decode('utf-8')
                    # Handle object arrays (often cell arrays in MATLAB)
                    elif isinstance(value, np.ndarray) and value.dtype == 'object':
                         # Check for HDF5 references (common for cell arrays)
                        if isinstance(value.flat[0], h5py.Reference):
                             data[k] = [f[ref][()] for ref in value.flat] # Dereference and load each cell
                        else:
                             # If not references, try loading as is (might need further handling)
                             data[k] = value
                    # Handle numeric types explicitly if needed (e.g., ensuring float)
                    elif np.issubdtype(value.dtype, np.number):
                         data[k] = value.astype(float) # Example: ensure float
                    else:
                        data[k] = value
                elif isinstance(v, h5py.Group):
                    # Recursively load subgroups (might represent structs)
                    data[k] = load_hdf5_group(v)
            return data

        mat_data = load_hdf5_group(f)

except Exception as e:
    # Add specific h5py error check if needed
    raise IOError(f"Error loading .mat file using h5py: {e}")

# --- Extract and Verify Variables ---
# Variables loaded from .mat files often need minor adjustments
required_vars = ['pitchRates', 'tiltAngles', 'totalRuns', 'runPitchRates', 'runTiltAngles',
                 'timeVec', 'nozzleHist', 'thetaHist',
                 'thetaCmdHist', 'maxHorizDistHist', 'maxAltitudeHist', 'maxAoAHist',
                 'aoaHist'] # Added tiltAngles, runPitchRates, runTiltAngles

# Check if all required variables are present
missing_vars = [var for var in required_vars if var not in mat_data]
if missing_vars:
     # Provide a more helpful error message if AoA is missing
     if 'maxAoAHist' in missing_vars and 'aoaHist' not in missing_vars:
         raise KeyError(f"The loaded results file {results_file_path} is missing 'maxAoAHist'. "
                        f"Please ensure 'pitch_rate_sweep.m' calculates and saves max absolute AoA.")
     elif 'aoaHist' in missing_vars and 'maxAoAHist' not in missing_vars:
         raise KeyError(f"The loaded results file {results_file_path} is missing 'aoaHist'. "
                        f"Please ensure 'pitch_rate_sweep.m' saves the full AoA history.")
     elif 'aoaHist' in missing_vars and 'maxAoAHist' in missing_vars:
          raise KeyError(f"The loaded results file {results_file_path} is missing 'aoaHist' and 'maxAoAHist'. "
                        f"Please ensure 'pitch_rate_sweep.m' saves the full AoA history and calculates max absolute AoA.")
     else:
        raise KeyError(f"Missing required variables in .mat file: {', '.join(missing_vars)}")

# Extract data (adjusting for potential scalar vs array issues from h5py)
pitch_rates_unique = np.sort(np.unique(np.atleast_1d(mat_data['pitchRates']).flatten())) # Unique sorted pitch rates
tilt_angles_unique = np.sort(np.unique(np.atleast_1d(mat_data['tiltAngles']).flatten())) # Unique sorted tilt angles
total_runs = int(mat_data['totalRuns']) # h5py might load scalars correctly, ensure int
run_pitch_rates = np.atleast_1d(mat_data['runPitchRates']).flatten() # PR for each run index
run_tilt_angles = np.atleast_1d(mat_data['runTiltAngles']).flatten() # Tilt for each run index
time_vec = np.atleast_1d(mat_data['timeVec']).flatten() # Ensure time_vec is 1D

# Cell arrays loaded via h5py might already be lists of arrays if handled by load_hdf5_group
nozzle_hist = mat_data['nozzleHist']
theta_hist = mat_data['thetaHist']
theta_cmd_hist = mat_data['thetaCmdHist']
aoa_hist = mat_data['aoaHist'] # Extract full AoA history

# These might be simple arrays
max_altitude_hist = np.atleast_1d(mat_data['maxAltitudeHist']).flatten()
max_horiz_dist_hist = np.atleast_1d(mat_data['maxHorizDistHist']).flatten()
max_aoa_hist = np.atleast_1d(mat_data['maxAoAHist']).flatten()  # Extract Max AoA
# --- DEBUG: Print maxAoAHist values for debugging ---
print("DEBUG maxAoAHist values:", max_aoa_hist)

# Ensure history arrays are correctly handled (adjust for h5py loading)
# The load_hdf5_group function aims to return lists of numpy arrays for cell arrays
if not isinstance(nozzle_hist, list):
     warnings.warn("nozzle_hist was not loaded as a list by h5py loader. Attempting conversion.")
     # Add conversion logic if needed, e.g., if it's a flat array that needs splitting
     # This depends heavily on how MATLAB saved the cell array in v7.3 format
     # For now, assume load_hdf5_group handled it or it's a single run case
     if total_runs == 1 and isinstance(nozzle_hist, np.ndarray):
         nozzle_hist = [nozzle_hist]
         theta_hist = [theta_hist]
         theta_cmd_hist = [theta_cmd_hist]
         aoa_hist = [aoa_hist]
     # Else: More complex handling might be needed here based on actual structure

# Ensure elements within the list are numpy arrays (flatten just in case)
if isinstance(nozzle_hist, list):
    nozzle_hist = [np.atleast_1d(arr).flatten() for arr in nozzle_hist]
    theta_hist = [np.atleast_1d(arr).flatten() for arr in theta_hist]
    theta_cmd_hist = [np.atleast_1d(arr).flatten() for arr in theta_cmd_hist]
    aoa_hist = [np.atleast_1d(arr).flatten() for arr in aoa_hist]
else:
     # This case should ideally be handled above or by load_hdf5_group
     warnings.warn("History arrays are not in the expected list format after h5py loading.")


print("All required variables extracted.")
print(f"Total runs: {total_runs}")
print(f"Unique Pitch rates: {pitch_rates_unique}")
print(f"Unique Tilt angles: {tilt_angles_unique}")
print(f"Shape of run_pitch_rates: {run_pitch_rates.shape}")
print(f"Shape of max_altitude_hist: {max_altitude_hist.shape}")

# --- Pre-calculate Max Nozzle Deflection --- (Moved up)
max_nozzle_abs = np.full(total_runs, np.nan)
for i in range(total_runs):
     if i < len(nozzle_hist) and isinstance(nozzle_hist[i], np.ndarray) and nozzle_hist[i].size > 0:
         max_nozzle_abs[i] = np.nanmax(np.abs(nozzle_hist[i]))

# --- Define Constraints --- (Moved up)
target_dist = 5.0
max_nozzle_limit = 4.0
aoa_limit = 10.0

# --- 2D Optimization: Find Optimal Combination --- (Replaces 1D optimization)
print("Finding optimal Tilt Angle and Pitch Rate combination...")
optimal_run_index = -1
optimal_tilt_angle = np.nan
optimal_pitch_rate = np.nan
max_valid_altitude = -np.inf

valid_run_indices = []

for i in range(total_runs):
    # Check for NaN results first
    is_nan_result = (
        np.isnan(max_altitude_hist[i]) or
        np.isnan(max_horiz_dist_hist[i]) or
        np.isnan(max_nozzle_abs[i]) or
        np.isnan(max_aoa_hist[i])
    )
    if is_nan_result:
        # print(f"Run {i} skipped due to NaN result.")
        continue

    # Check constraints
    dist_ok = max_horiz_dist_hist[i] >= target_dist
    nozzle_ok = max_nozzle_abs[i] <= max_nozzle_limit
    aoa_ok = max_aoa_hist[i] <= aoa_limit

    if dist_ok and nozzle_ok and aoa_ok:
        valid_run_indices.append(i)
        # Check if this valid run has higher altitude
        if max_altitude_hist[i] > max_valid_altitude:
            max_valid_altitude = max_altitude_hist[i]
            optimal_run_index = i
            optimal_tilt_angle = run_tilt_angles[i]
            optimal_pitch_rate = run_pitch_rates[i]

if optimal_run_index != -1:
    print(f"Optimal combination found (Run Index {optimal_run_index}):")
    print(f"  Tilt Angle: {optimal_tilt_angle:.1f}°")
    print(f"  Pitch Rate: {optimal_pitch_rate:.2f}°/s")
    print(f"  Max Altitude: {max_valid_altitude:.2f} km")
    print(f"  (Satisfies Dist >= {target_dist}km, |Nozzle| <= {max_nozzle_limit}°, |AoA| <= {aoa_limit}°)")
else:
    print(f"No combination found satisfying all constraints (Dist >= {target_dist}km, |Nozzle| <= {max_nozzle_limit}°, |AoA| <= {aoa_limit}°). Cannot determine optimum.")

# --- Remove Old Interpolation and Bracketing Logic --- (Delete these sections)
# Delete the blocks titled:
# "Interpolation and Optimal Calculation (Moved Before Plot 1)"
# "Find Bracketing Pitch Rate Indices"
# "Calculate Interpolated Optimal Time Histories"

# --- Reshape Data for 2D Plotting --- (New Section)
print("Reshaping data for 2D plots...")
n_tilts = len(tilt_angles_unique)
n_rates = len(pitch_rates_unique)

# Create meshgrid for plotting
TILT, PR = np.meshgrid(tilt_angles_unique, pitch_rates_unique, indexing='ij')

# Initialize 2D arrays with NaNs
ALT_2D = np.full((n_tilts, n_rates), np.nan)
DIST_2D = np.full((n_tilts, n_rates), np.nan)
NOZZLE_2D = np.full((n_tilts, n_rates), np.nan)
AOA_2D = np.full((n_tilts, n_rates), np.nan)

# Map 1D results to 2D grid
for i in range(total_runs):
    try:
        tilt_idx = np.where(tilt_angles_unique == run_tilt_angles[i])[0][0]
        rate_idx = np.where(pitch_rates_unique == run_pitch_rates[i])[0][0]

        ALT_2D[tilt_idx, rate_idx] = max_altitude_hist[i]
        DIST_2D[tilt_idx, rate_idx] = max_horiz_dist_hist[i]
        NOZZLE_2D[tilt_idx, rate_idx] = max_nozzle_abs[i]
        AOA_2D[tilt_idx, rate_idx] = max_aoa_hist[i]
    except IndexError:
        warnings.warn(f"Could not map run {i} (Tilt={run_tilt_angles[i]}, PR={run_pitch_rates[i]}) to 2D grid. Skipping.")

print("Data reshaped.")

# --- Interpolation Setup --- (New Section)
print("Setting up 2D interpolators...")
# Need points for interpolation axes - use the unique sorted values
points = (tilt_angles_unique, pitch_rates_unique)

# Function to create interpolator, handling potential NaNs by filling with a value
# that likely won't be chosen (e.g., very low altitude, very high constraint values)
def create_interpolator(values_2d, fill_value_strategy='min'):
    values_copy = values_2d.copy()
    if np.isnan(values_copy).any():
        warnings.warn(f"NaNs found in data for interpolation. Filling NaNs.")
        if fill_value_strategy == 'min':
            fill_val = np.nanmin(values_copy) - 1e6 # Fill with very small value
        elif fill_value_strategy == 'max':
             fill_val = np.nanmax(values_copy) + 1e6 # Fill with very large value
        else: # default to a reasonable large negative number for altitude, large positive for constraints
             fill_val = -1e9 if fill_value_strategy == 'altitude' else 1e9
        values_copy[np.isnan(values_copy)] = fill_val
    try:
        # bounds_error=False prevents errors if optimizing near edge, fill_value=None uses nearest for extrapolation
        return RegularGridInterpolator(points, values_copy, method='linear', bounds_error=False, fill_value=None)
    except ValueError as e:
         print(f"Error creating interpolator: {e}")
         print("Check if grid points (tilt_angles_unique, pitch_rates_unique) are strictly increasing.")
         # Fallback: return a function that always returns NaN or the fill value
         error_fill = -np.inf if fill_value_strategy == 'altitude' else np.inf
         return lambda x: np.full(np.shape(x)[0] if len(np.shape(x)) > 1 else 1, error_fill)


interp_alt = create_interpolator(ALT_2D, fill_value_strategy='altitude')
interp_dist = create_interpolator(DIST_2D, fill_value_strategy='min')
interp_nozzle = create_interpolator(NOZZLE_2D, fill_value_strategy='max')
interp_aoa = create_interpolator(AOA_2D, fill_value_strategy='max')

print("Interpolators created.")


# --- 2D Optimization with Interpolation --- (New Section)
print("Performing optimization using interpolation...")

# Define the objective function to minimize (negative altitude)
def objective_function(params):
    tilt, pr = params
    point = np.array([[tilt, pr]]) # Interpolator expects points as (n_points, n_dims)

    # Interpolate performance metrics
    alt = interp_alt(point)[0]
    dist = interp_dist(point)[0]
    nozzle = interp_nozzle(point)[0]
    aoa = interp_aoa(point)[0]

    # Check if interpolation/extrapolation resulted in NaN
    if np.isnan(alt) or np.isnan(dist) or np.isnan(nozzle) or np.isnan(aoa):
         # This might happen if the nearest point used for extrapolation
         # originally had a NaN value that wasn't properly filled,
         # or if the interpolation itself fails numerically.
         warnings.warn(f"Interpolator returned NaN for point {params}. Penalizing.")
         return np.inf # Penalize points where interpolation failed

    # Check constraints
    dist_ok = dist >= target_dist
    nozzle_ok = nozzle <= max_nozzle_limit
    aoa_ok = aoa <= aoa_limit

    if dist_ok and nozzle_ok and aoa_ok:
        return -alt # Minimize negative altitude
    else:
        # Return infinity if constraints are violated
        # Could add penalty based on violation magnitude here if desired
        return np.inf

# Define bounds for the optimization variables
bounds = [(np.min(tilt_angles_unique), np.max(tilt_angles_unique)),
          (np.min(pitch_rates_unique), np.max(pitch_rates_unique))]

# Initial guess: Use the best discrete point found earlier
initial_guess = [optimal_tilt_angle, optimal_pitch_rate] if optimal_run_index != -1 else [np.mean(bounds[0]), np.mean(bounds[1])]

# Perform the optimization
opt_result = minimize(objective_function,
                      initial_guess,
                      method='L-BFGS-B', # Method that handles bounds
                      bounds=bounds,
                      options={'disp': False, 'eps': 1e-3}) # Adjust tolerance if needed

optimal_tilt_angle_interp = np.nan
optimal_pitch_rate_interp = np.nan
optimal_altitude_interp = np.nan
optimal_dist_interp = np.nan
optimal_nozzle_interp = np.nan
optimal_aoa_interp = np.nan

if opt_result.success and np.isfinite(opt_result.fun):
    optimal_tilt_angle_interp, optimal_pitch_rate_interp = opt_result.x
    # Recalculate metrics at the found optimum using interpolators
    opt_point = np.array([[optimal_tilt_angle_interp, optimal_pitch_rate_interp]])
    optimal_altitude_interp = interp_alt(opt_point)[0]
    optimal_dist_interp = interp_dist(opt_point)[0]
    optimal_nozzle_interp = interp_nozzle(opt_point)[0]
    optimal_aoa_interp = interp_aoa(opt_point)[0]

    print("\n--- Interpolated Optimal Result ---")
    print(f"  Tilt Angle: {optimal_tilt_angle_interp:.2f}°")
    print(f"  Pitch Rate: {optimal_pitch_rate_interp:.3f}°/s")
    print(f"  Interpolated Max Altitude: {optimal_altitude_interp:.3f} km")
    print(f"  Interpolated Max Distance: {optimal_dist_interp:.3f} km")
    print(f"  Interpolated Max |Nozzle|: {optimal_nozzle_interp:.3f}°")
    print(f"  Interpolated Max |AoA|: {optimal_aoa_interp:.3f}°")
    print(f"  (Constraints: Dist >= {target_dist}km, |Nozzle| <= {max_nozzle_limit}°, |AoA| <= {aoa_limit}°)")

    # Compare with best discrete point
    print("\n--- Best Discrete Run (for comparison) ---")
    if optimal_run_index != -1:
        print(f"  Run Index: {optimal_run_index}")
        print(f"  Tilt Angle: {optimal_tilt_angle:.1f}°")
        print(f"  Pitch Rate: {optimal_pitch_rate:.2f}°/s")
        print(f"  Max Altitude: {max_valid_altitude:.2f} km")
    else:
        print("  No valid discrete run found.")

else:
    print("\nOptimization failed or did not find a valid point satisfying constraints.")
    print(f"Optimization status: {opt_result.message}")
    # Keep using the best discrete point if optimization fails
    optimal_tilt_angle_interp = optimal_tilt_angle
    optimal_pitch_rate_interp = optimal_pitch_rate


# --- Plot 1: Nozzle Angle Time Histories --- (Modified)
print("Generating Plot 1: Nozzle Histories...")
fig1, ax1 = plt.subplots(figsize=(12, 7))

# Define linestyles for different tilt angles
linestyles = ['-', '--', ':', '-.']
tilt_style_map = {tilt: linestyles[i % len(linestyles)] for i, tilt in enumerate(tilt_angles_unique)}

# Define colormap for pitch rates
colors1 = plt.cm.viridis
norm1 = Normalize(vmin=np.nanmin(pitch_rates_unique), vmax=np.nanmax(pitch_rates_unique))
sm1 = plt.cm.ScalarMappable(cmap=colors1, norm=norm1)
sm1.set_array([]) # Needed for colorbar

plot_count1 = 0
for i in range(total_runs):
    # Check if data exists and is valid for this run
    time_vec_valid = time_vec.size > 0
    if (time_vec_valid and
        i < len(nozzle_hist) and isinstance(nozzle_hist[i], np.ndarray) and nozzle_hist[i].size == time_vec.size and not np.all(np.isnan(nozzle_hist[i]))):

        current_pr = run_pitch_rates[i]
        current_tilt = run_tilt_angles[i]
        color = colors1(norm1(current_pr))
        linestyle = tilt_style_map.get(current_tilt, '-')

        # Determine alpha and linewidth based on constraints for THIS run
        is_valid_run = i in valid_run_indices
        is_optimal_run = (i == optimal_run_index)

        alpha = 1.0 if is_valid_run else 0.2 # Dim invalid runs
        linewidth = 2.5 if is_optimal_run else (1.5 if is_valid_run else 1.0) # Make optimal slightly thicker
        zorder = 10 if is_optimal_run else (5 if is_valid_run else 1)
        plot_color = 'red' if is_optimal_run else color # Use bright red for optimal run

        label = None
        # Avoid cluttering legend - maybe add labels only for optimal or specific runs if needed
        # label = f'T={current_tilt:.0f}, PR={current_pr:.1f}' # Example if labeling all

        ax1.plot(time_vec, nozzle_hist[i], color=plot_color, linestyle=linestyle, alpha=alpha, linewidth=linewidth, label=label, zorder=zorder)
        plot_count1 += 1
    else:
        # More specific feedback
        reason = []
        if not time_vec_valid: reason.append("invalid time_vec")
        if not (i < len(nozzle_hist) and isinstance(nozzle_hist[i], np.ndarray)): reason.append("invalid nozzle_hist")
        elif not (nozzle_hist[i].size == time_vec.size): reason.append("nozzle_hist size mismatch")
        elif np.all(np.isnan(nozzle_hist[i])): reason.append("nozzle_hist all NaN")
        print(f"Skipping plot 1 for run {i+1} due to: {', '.join(reason) if reason else 'unknown issue'}.")

# Remove bracketing fill_between
# Remove interpolated optimal plot

if plot_count1 > 0:
    # Add horizontal lines for nozzle limits
    ax1.axhline(max_nozzle_limit, color='grey', linestyle='--', linewidth=1.5, label=f'Nozzle Limit ({max_nozzle_limit}°)')
    ax1.axhline(-max_nozzle_limit, color='grey', linestyle='--', linewidth=1.5)

    # Add colorbar for pitch rate
    cbar1 = fig1.colorbar(sm1, ax=ax1)
    cbar1.set_label('Initial Pitch Rate (°/s)')

    # Create custom legend for linestyles (tilt angles)
    legend_elements = [Line2D([0], [0], color='grey', lw=1.5, linestyle=style, label=f'Tilt {tilt:.0f}°')
                       for tilt, style in tilt_style_map.items()]
    # Add optimal run label if exists
    if optimal_run_index != -1:
         # Label uses the *interpolated* values, but highlights the *best discrete* run's line
         optimal_label = (f'Optimal (Interp: T={optimal_tilt_angle_interp:.1f}°, PR={optimal_pitch_rate_interp:.2f}°/s)\n'
                          f'        (Best Discrete: T={optimal_tilt_angle:.1f}°, PR={optimal_pitch_rate:.2f}°/s)')
         legend_elements.append(Line2D([0], [0], color='red', lw=2.5, linestyle=tilt_style_map.get(optimal_tilt_angle, '-'), # Use red color in legend
                                       label=optimal_label))
    # Add limit line label
    legend_elements.append(Line2D([0], [0], color='grey', linestyle='--', linewidth=1.5, label=f'Nozzle Limit ({max_nozzle_limit}°)'))

    ax1.legend(handles=legend_elements, loc='upper right', fontsize='small')

else:
    print("No valid data to plot in Plot 1.")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Nozzle Angle (°)")
ax1.set_title("Nozzle Angle vs Time (Color=Pitch Rate, Style=Tilt Angle)")
ax1.grid(True, which='both', linestyle='--', linewidth=0.5)


# --- Plot 2: Theta and Theta Cmd Time Histories --- (Modified)
print("Generating Plot 2: Theta and Theta Cmd Histories...")
fig2, ax2 = plt.subplots(figsize=(12, 7))

# Use same linestyles and colormap as Plot 1
norm2 = Normalize(vmin=np.nanmin(pitch_rates_unique), vmax=np.nanmax(pitch_rates_unique))
sm2 = plt.cm.ScalarMappable(cmap=colors1, norm=norm2)
sm2.set_array([])

plot_count2 = 0
skip_initial_points = 5 # Number of initial data points to skip

for i in range(total_runs):
    # Check if data exists and is valid for this run
    if i < len(theta_hist) and isinstance(theta_hist[i], np.ndarray) and theta_hist[i].size > skip_initial_points and not np.all(np.isnan(theta_hist[i])):
        if len(time_vec) > skip_initial_points:
            time_vec_sliced = time_vec[skip_initial_points:]
            theta_hist_sliced = theta_hist[i][skip_initial_points:]
            theta_cmd_hist_sliced = theta_cmd_hist[i][skip_initial_points:] if i < len(theta_cmd_hist) and isinstance(theta_cmd_hist[i], np.ndarray) and theta_cmd_hist[i].size > skip_initial_points else None

            if len(time_vec_sliced) == len(theta_hist_sliced):
                current_pr = run_pitch_rates[i]
                current_tilt = run_tilt_angles[i]
                color = colors1(norm2(current_pr))
                linestyle = tilt_style_map.get(current_tilt, '-')

                is_valid_run = i in valid_run_indices
                is_optimal_run = (i == optimal_run_index)

                alpha = 1.0 if is_valid_run else 0.2
                linewidth = 2.5 if is_optimal_run else (1.5 if is_valid_run else 1.0)
                zorder = 10 if is_optimal_run else (5 if is_valid_run else 1)
                plot_color = 'red' if is_optimal_run else color # Ensure this line is present and correct

                label = None

                # Plot Theta (actual) using plot_color
                ax2.plot(time_vec_sliced, theta_hist_sliced, color=plot_color, linestyle=linestyle, alpha=alpha, linewidth=linewidth, label=label, zorder=zorder)

                # Plot Theta Cmd (lighter) using original color
                if theta_cmd_hist_sliced is not None and len(time_vec_sliced) == len(theta_cmd_hist_sliced) and not np.all(np.isnan(theta_cmd_hist_sliced)):
                    ax2.plot(time_vec_sliced, theta_cmd_hist_sliced, color=color, linestyle=linestyle, alpha=0.3, linewidth=1.0, zorder=zorder-1)
                # else: # Optional: Add warning if cmd hist is skipped
                #     print(f"Skipping theta_cmd plot for run {i+1} due to invalid data or size mismatch.")

                plot_count2 += 1
            else:
                warnings.warn(f"Skipping run {i} in Plot 2 due to time/theta length mismatch after slicing.")
        else:
             warnings.warn(f"Skipping run {i} in Plot 2 due to insufficient time vector length (need > {skip_initial_points} points).")
    else:
         # This warning is now potentially redundant with the checks inside, but keep for now
         # warnings.warn(f"Skipping run {i} in Plot 2 due to missing, invalid, or insufficient theta data (need > {skip_initial_points} points).")
         pass # Avoid duplicate warnings

# Remove bracketing fill_between
# Remove interpolated optimal plot

if plot_count2 > 0:
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Theta (°)")
    ax2.set_title("Pitch Angle (Theta) vs Time (Color=Pitch Rate, Style=Tilt Angle)")

    # Add colorbar and custom legend
    cbar2 = fig2.colorbar(sm2, ax=ax2)
    cbar2.set_label('Initial Pitch Rate (°/s)')
    legend_elements_2 = [Line2D([0], [0], color='grey', lw=1.5, linestyle=style, label=f'Tilt {tilt:.0f}°')
                         for tilt, style in tilt_style_map.items()]
    if optimal_run_index != -1:
         # Label uses the *interpolated* values, but highlights the *best discrete* run's line
         optimal_label_2 = (f'Optimal (Interp: T={optimal_tilt_angle_interp:.1f}°, PR={optimal_pitch_rate_interp:.2f}°/s)\n'
                            f'         (Best Discrete: T={optimal_tilt_angle:.1f}°, PR={optimal_pitch_rate:.2f}°/s)')
         legend_elements_2.append(Line2D([0], [0], color='red', lw=2.5, linestyle=tilt_style_map.get(optimal_tilt_angle, '-'), # Use red color in legend
                                         label=optimal_label_2))
    # Add label for command history
    legend_elements_2.append(Line2D([0], [0], color='grey', alpha=0.5, lw=1.0, linestyle='-', label='Theta Cmd (lighter)'))

    ax2.legend(handles=legend_elements_2, loc='best', fontsize='small')
    ax2.grid(True, which='both', linestyle='--', linewidth=0.5)
else:
    ax2.text(0.5, 0.5, 'No valid theta data to plot', horizontalalignment='center', verticalalignment='center', transform=ax2.transAxes)
    ax2.set_title("Theta and Theta Cmd vs Time (No valid data)")

# --- Plot 4: Performance Summary (Line Plots) --- (Replaces Contour Plots)
print("Generating Plot 4: Performance Summary (Line Plots)...")
fig4, axs4 = plt.subplots(2, 2, figsize=(12, 11), sharex=True) # Share X axis (Pitch Rate)
fig4.suptitle('Rocket Performance vs. Initial Pitch Rate (Colored by Tilt Angle)', fontsize=16)

# Define colors for different tilt angles
colors_tilt = plt.cm.viridis(np.linspace(0, 1, len(tilt_angles_unique)))
tilt_color_map = {tilt: colors_tilt[i] for i, tilt in enumerate(tilt_angles_unique)}

# Helper function to plot data for one tilt angle on a given axis
def plot_metric_vs_pr(ax, tilt, metric_hist, ylabel, title, constraint_val=None, constraint_type=None):
    indices = np.where(run_tilt_angles == tilt)[0]
    if len(indices) == 0:
        return # No data for this tilt

    pitch_rates = run_pitch_rates[indices]
    metric_values = metric_hist[indices]

    # Remove NaNs before sorting/plotting
    valid_mask = ~np.isnan(pitch_rates) & ~np.isnan(metric_values)
    pitch_rates = pitch_rates[valid_mask]
    metric_values = metric_values[valid_mask]

    if len(pitch_rates) == 0:
        return # No valid data after NaN removal

    sort_order = np.argsort(pitch_rates)
    pitch_rates_sorted = pitch_rates[sort_order]
    metric_values_sorted = metric_values[sort_order]

    color = tilt_color_map[tilt]
    ax.plot(pitch_rates_sorted, metric_values_sorted, marker='.', linestyle='-', color=color, label=f'Tilt {tilt:.0f}°')

    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.7)

    # Add constraint line if applicable
    if constraint_val is not None:
        line_color = 'red' if constraint_type == 'max' else 'lime'
        line_style = '--'
        # Only add label if it hasn't been added to this axis yet
        if f'limit_{constraint_val}' not in getattr(ax, '_added_labels', set()):
             ax.axhline(constraint_val, color=line_color, linestyle=line_style, linewidth=1.5, label=f'Limit ({constraint_val:.1f})')
             if not hasattr(ax, '_added_labels'): ax._added_labels = set()
             ax._added_labels.add(f'limit_{constraint_val}')
        else:
             ax.axhline(constraint_val, color=line_color, linestyle=line_style, linewidth=1.5)


# Plot data for each tilt angle on each subplot
for tilt in tilt_angles_unique:
    plot_metric_vs_pr(axs4[0, 0], tilt, max_altitude_hist, 'Max Altitude (km)', 'Max Altitude')
    plot_metric_vs_pr(axs4[0, 1], tilt, max_horiz_dist_hist, 'Max Distance (km)', 'Max Horizontal Distance', constraint_val=target_dist, constraint_type='min')
    plot_metric_vs_pr(axs4[1, 0], tilt, max_nozzle_abs, 'Max |Nozzle Angle| (°)', 'Max |Nozzle Angle|', constraint_val=max_nozzle_limit, constraint_type='max')
    plot_metric_vs_pr(axs4[1, 1], tilt, max_aoa_hist, 'Max |Angle of Attack| (°)', 'Max |Angle of Attack|', constraint_val=aoa_limit, constraint_type='max')

# Add common X label
axs4[1, 0].set_xlabel('Initial Pitch Rate (°/s)')
axs4[1, 1].set_xlabel('Initial Pitch Rate (°/s)')

# Mark optimal points
optimal_markers = []
# Mark interpolated optimal point
if np.isfinite(optimal_pitch_rate_interp):
     axs4[0, 0].plot(optimal_pitch_rate_interp, optimal_altitude_interp, 'X', color='red', markersize=12, markeredgecolor='white', label='Interpolated Optimal', zorder=10)
     axs4[0, 1].plot(optimal_pitch_rate_interp, optimal_dist_interp, 'X', color='red', markersize=12, markeredgecolor='white', zorder=10)
     axs4[1, 0].plot(optimal_pitch_rate_interp, optimal_nozzle_interp, 'X', color='red', markersize=12, markeredgecolor='white', zorder=10)
     axs4[1, 1].plot(optimal_pitch_rate_interp, optimal_aoa_interp, 'X', color='red', markersize=12, markeredgecolor='white', zorder=10)
     optimal_markers.append(Line2D([0], [0], marker='X', color='red', markersize=10, markeredgecolor='white', linestyle='None', label=f'Interpolated Optimal\n(PR={optimal_pitch_rate_interp:.2f}°/s, T={optimal_tilt_angle_interp:.1f}°)')) # Add tilt info

# Mark best discrete point
if optimal_run_index != -1:
     best_discrete_pr = optimal_pitch_rate
     best_discrete_tilt = optimal_tilt_angle
     best_discrete_alt = max_altitude_hist[optimal_run_index]
     best_discrete_dist = max_horiz_dist_hist[optimal_run_index]
     best_discrete_nozzle = max_nozzle_abs[optimal_run_index]
     best_discrete_aoa = max_aoa_hist[optimal_run_index]

     # Check if the optimal discrete point has valid data before plotting
     if np.isfinite(best_discrete_pr) and np.isfinite(best_discrete_tilt):
         # Find the correct axes and plot
         axs4[0, 0].plot(best_discrete_pr, best_discrete_alt, 'o', color=tilt_color_map[best_discrete_tilt], markersize=8, markeredgecolor='black', label='Best Discrete', zorder=9)
         axs4[0, 1].plot(best_discrete_pr, best_discrete_dist, 'o', color=tilt_color_map[best_discrete_tilt], markersize=8, markeredgecolor='black', zorder=9)
         axs4[1, 0].plot(best_discrete_pr, best_discrete_nozzle, 'o', color=tilt_color_map[best_discrete_tilt], markersize=8, markeredgecolor='black', zorder=9)
         axs4[1, 1].plot(best_discrete_pr, best_discrete_aoa, 'o', color=tilt_color_map[best_discrete_tilt], markersize=8, markeredgecolor='black', zorder=9)
         optimal_markers.append(Line2D([0], [0], marker='o', color=tilt_color_map[best_discrete_tilt], markersize=8, markeredgecolor='black', linestyle='None', label=f'Best Discrete\n(PR={best_discrete_pr:.2f}°/s, T={best_discrete_tilt:.1f}°)'))

# Create legend handles for tilt angles
tilt_handles = [Line2D([0], [0], color=tilt_color_map[tilt], lw=2, label=f'Tilt {tilt:.0f}°') for tilt in tilt_angles_unique]

# Combine legends: Tilt angles first, then optimal markers
all_handles = tilt_handles + optimal_markers
# Add constraint handles (need to create them manually as axhline doesn't return a handle easily for figure legend)
constraint_handles = []
if target_dist is not None: constraint_handles.append(Line2D([0], [0], color='lime', linestyle='--', linewidth=1.5, label=f'Dist Limit ({target_dist:.1f})'))
if max_nozzle_limit is not None: constraint_handles.append(Line2D([0], [0], color='red', linestyle='--', linewidth=1.5, label=f'Nozzle Limit ({max_nozzle_limit:.1f})'))
if aoa_limit is not None: constraint_handles.append(Line2D([0], [0], color='red', linestyle='--', linewidth=1.5, label=f'AoA Limit ({aoa_limit:.1f})'))

all_handles.extend(constraint_handles)

# Place legend below the subplots
# Adjust ncol dynamically based on number of handles
num_legend_items = len(all_handles)
ncol_legend = min(num_legend_items, 4) # Max 4 columns, adjust as needed
fig4.legend(handles=all_handles, loc='lower center', ncol=ncol_legend, bbox_to_anchor=(0.5, 0.01), fontsize='small')

# Adjust layout to prevent overlap and make space for legend
plt.tight_layout(rect=[0, 0.08, 1, 0.95]) # Adjust rect bottom (0.08) and top (0.95)

if save_plots:
    save_path = os.path.join(results_dir, 'py_sweep_performance_lines.png') # New filename
    fig4.savefig(save_path, dpi=300)
    print(f"Plot 4 saved to {save_path}")
else:
    print("Plot 4 generated (displaying).")


# --- Plot 5: Angle of Attack Time Histories --- (Modified)
print("Generating Plot 5: Angle of Attack Histories...")
fig5, ax5 = plt.subplots(figsize=(12, 7))

# Use same linestyles and colormap as Plot 1 & 2
norm5 = Normalize(vmin=np.nanmin(pitch_rates_unique), vmax=np.nanmax(pitch_rates_unique))
sm5 = plt.cm.ScalarMappable(cmap=colors1, norm=norm5) # Use colors1 (viridis) and norm5
sm5.set_array([])

plot_count5 = 0
skip_initial_points = 5 # Number of initial data points to skip

for i in range(total_runs):
    # Check if data exists and is valid for this run, considering skipping
    if i < len(aoa_hist) and isinstance(aoa_hist[i], np.ndarray) and aoa_hist[i].size > skip_initial_points and not np.all(np.isnan(aoa_hist[i])):
        if len(time_vec) > skip_initial_points:
            time_vec_sliced = time_vec[skip_initial_points:]
            aoa_hist_sliced = aoa_hist[i][skip_initial_points:]

            if len(time_vec_sliced) == len(aoa_hist_sliced):
                current_pr = run_pitch_rates[i]
                current_tilt = run_tilt_angles[i]
                color = colors1(norm5(current_pr)) # Use norm5 here
                linestyle = tilt_style_map.get(current_tilt, '-')

                is_valid_run = i in valid_run_indices
                is_optimal_run = (i == optimal_run_index)

                alpha = 1.0 if is_valid_run else 0.2
                linewidth = 2.5 if is_optimal_run else (1.5 if is_valid_run else 1.0)
                zorder = 10 if is_optimal_run else (5 if is_valid_run else 1)
                plot_color = 'red' if is_optimal_run else color # Ensure this line is present and correct

                label = None

                # Plot AoA using plot_color and sliced data
                ax5.plot(time_vec_sliced, aoa_hist_sliced, color=plot_color, linestyle=linestyle, alpha=alpha, linewidth=linewidth, label=label, zorder=zorder)
                plot_count5 += 1
            else:
                warnings.warn(f"Skipping run {i} in Plot 5 due to time/AoA length mismatch after slicing.")
        else:
             warnings.warn(f"Skipping run {i} in Plot 5 due to insufficient time vector length (need > {skip_initial_points} points).")
    else:
        # More specific feedback if needed
        pass # Avoid duplicate warnings

# Remove bracketing fill_between
# Remove interpolated optimal plot

if plot_count5 > 0:
    # Add horizontal lines for AoA limits
    ax5.axhline(aoa_limit, color='grey', linestyle='--', linewidth=1.5, label=f'AoA Limit ({aoa_limit}°)')
    ax5.axhline(-aoa_limit, color='grey', linestyle='--', linewidth=1.5)

    # Add colorbar and custom legend
    cbar5 = fig5.colorbar(sm5, ax=ax5)
    cbar5.set_label('Initial Pitch Rate (°/s)')
    legend_elements_5 = [Line2D([0], [0], color='grey', lw=1.5, linestyle=style, label=f'Tilt {tilt:.0f}°')
                         for tilt, style in tilt_style_map.items()]
    if optimal_run_index != -1:
         # Label uses the *interpolated* values, but highlights the *best discrete* run's line
         optimal_label_5 = (f'Optimal (Interp: T={optimal_tilt_angle_interp:.1f}°, PR={optimal_pitch_rate_interp:.2f}°/s)\n'
                            f'         (Best Discrete: T={optimal_tilt_angle:.1f}°, PR={optimal_pitch_rate:.2f}°/s)')
         legend_elements_5.append(Line2D([0], [0], color='red', lw=2.5, linestyle=tilt_style_map.get(optimal_tilt_angle, '-'), # Use red color in legend
                                         label=optimal_label_5))
    # Add limit line label
    legend_elements_5.append(Line2D([0], [0], color='grey', linestyle='--', linewidth=1.5, label=f'AoA Limit ({aoa_limit}°)'))

    ax5.legend(handles=legend_elements_5, loc='upper right', fontsize='small')
else:
    print("No valid data to plot in Plot 5.")

ax5.set_xlabel("Time (s)")
ax5.set_ylabel("Angle of Attack (°)")
ax5.set_title("Angle of Attack vs Time (Color=Pitch Rate, Style=Tilt Angle)")
ax5.grid(True, which='both', linestyle='--', linewidth=0.5)


# --- Display Plots ---
print("\nDisplaying plots...")
plt.show()

print("\nPython plotting script finished.")
