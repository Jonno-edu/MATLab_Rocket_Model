# FULL REVISED CODE - Minimal GUI (Play/Pause Only) + Threading

import sys
import pyvista as pv
import pyvistaqt as pvqt
import pandas as pd
import numpy as np
import math
import time
import collections
import threading # For pausing event

# Import necessary PyQt components
try:
    # Try PyQt6 first
    from PyQt6.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                                 QWidget, QPushButton, QLabel, QFrame, QSizePolicy) # Added QSizePolicy
    from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
    print("Using PyQt6")
    pyqt_version = 6
except ImportError:
    try:
        # Fallback to PyQt5
        from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout,
                                     QWidget, QPushButton, QLabel, QFrame, QSizePolicy) # Added QSizePolicy
        from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QThread
        print("Using PyQt5")
        pyqt_version = 5
    except ImportError:
        print("ERROR: PyQt5 or PyQt6 is required.")
        sys.exit(1)


# --- Allow Empty Meshes ---
pv.global_theme.allow_empty_mesh = True

# --- Configuration ---
CSV_FILENAME = 'rocket_trajectory_100fps.csv'
ROCKET_HEIGHT = 5.0
ROCKET_RADIUS = 0.5
NOZZLE_HEIGHT_FACTOR = 0.15
NOZZLE_RADIUS_FACTOR = 0.7
THRUST_LENGTH_SCALE = 5.0
THRUST_COLOR = 'orange'
ROCKET_COLOR = 'silver'
NOZZLE_COLOR = 'darkgrey'
BACKGROUND_COLOR = 'black'
TRAJECTORY_COLOR = 'cyan'
GRID_COLOR = 'grey'
ANIMATION_SPEED_FACTOR = 1.0 # Keep for potential future use or internal logic

# --- Performance Tuning ---
ENABLE_PERFORMANCE_TIMING = True
PRINT_TIMING_FREQUENCY = 30   # Print timing info roughly every second
TARGET_DISPLAY_FPS = 30.0     # Target visual frame rate
FRAME_DURATION_SEC = 1.0 / TARGET_DISPLAY_FPS # Time per frame
USE_LIMITED_TRAIL = True
TRAIL_DURATION_SECONDS = 5.0  # Keep trail short
DISABLE_ANTI_ALIASING = True
INTERPOLATE_FRAMES = True      # Keep interpolation

# --- Grid, Earth and Camera Configuration ---
GRID_BOUNDS = [-500, 500, -10, 10, -50, 2000]
EARTH_VISUAL_RADIUS = 1500.0
EARTH_CENTER = (0.0, 0.0, -EARTH_VISUAL_RADIUS)
INITIAL_CAM_OFFSET = np.array([-40.0, -40.0, 15.0])
# Reduced Geometry
EARTH_THETA_RES = 30
EARTH_PHI_RES = 30
ROCKET_RESOLUTION = 16
NOZZLE_RESOLUTION = 12
# Camera Follow (Fixed)
follow_camera_active = True

# --- Helper Function ---
def load_data(filename):
    try:
        df=pd.read_csv(filename);required_cols=['t','X','Y','Z','theta','nozzleAngle']
        if not all(col in df.columns for col in required_cols):raise ValueError(f"CSV must contain columns: {', '.join(required_cols)}")
        print(f"Successfully loaded data from {filename}. Rows: {len(df)}")
        data={'t':df['t'].to_numpy(),'pos':np.stack((df['X'],df['Y'] if 'Y' in df.columns else np.zeros(len(df)),df['Z']),axis=-1),'theta':df['theta'].to_numpy(),'nozzle_angle':df['nozzleAngle'].to_numpy()}
        if len(data['t'])>1:data['avg_dt']=np.median(np.diff(data['t']));data['total_duration']=data['t'][-1]-data['t'][0]
        else:data['avg_dt']=0.01;data['total_duration']=0.0
        if data['avg_dt']<=0:data['avg_dt']=0.01
        return data
    except FileNotFoundError:print(f"Error: CSV file not found at '{filename}'");return None
    except Exception as e:print(f"Error loading CSV data: {e}");return None

# --- Animation Worker Thread ---
class AnimationWorker(QObject):
    update_needed = pyqtSignal(float) # Emits target_time
    # progress_update = pyqtSignal(int) # REMOVED
    finished = pyqtSignal()

    # Removed speed_factor argument from init
    def __init__(self, data, frame_duration_sec):
        super().__init__()
        self.data = data
        self._speed_factor = ANIMATION_SPEED_FACTOR # Use global constant
        self._frame_duration_sec = frame_duration_sec
        self._running = False
        self._paused = False
        self.pause_event = threading.Event()

    # Removed set_speed method

    def run(self):
        self._running = True
        self._paused = False
        self.pause_event.set()

        start_sim_time = self.data['t'][0]
        end_sim_time = self.data['t'][-1]
        # total_duration = self.data['total_duration'] # Not needed in worker anymore
        # scrub_slider_max = 1000 # Not needed

        start_real_time = time.perf_counter()
        frame_counter = 0
        # progress_update_counter = 0 # REMOVED
        # progress_update_interval = int(TARGET_DISPLAY_FPS / 5) # REMOVED

        print("Animation worker started.")
        while self._running:
            self.pause_event.wait()
            if not self._running: break

            frame_start_time = time.perf_counter()
            elapsed_real_time = frame_start_time - start_real_time
            target_time = start_sim_time + elapsed_real_time * self._speed_factor

            if target_time >= end_sim_time:
                target_time = end_sim_time
                self._running = False

            self.update_needed.emit(target_time)

            # REMOVED Progress Update Logic

            target_next_frame_real_time = start_real_time + (frame_counter + 1) * self._frame_duration_sec
            current_real_time = time.perf_counter()
            time_to_wait = target_next_frame_real_time - current_real_time
            if time_to_wait > 0.001: time.sleep(time_to_wait)

            frame_counter += 1

        print("Animation worker finished.")
        self.finished.emit()

    def pause(self):
        self._paused = True
        self.pause_event.clear(); print("Animation worker paused.")

    def resume(self):
        self._paused = False
        self.pause_event.set(); print("Animation worker resumed.")

    def stop(self):
        self._running = False
        self.pause_event.set(); print("Animation worker stop requested.")


# --- Main Application Window ---
class RocketAnimatorApp(QMainWindow):

    def __init__(self, data):
        super().__init__()
        self.data = data
        if self.data is None or len(self.data['t']) < 2: sys.exit(1)

        self.num_data_points = len(self.data['t'])
        self.total_duration = self.data['total_duration']
        # self.is_playing state managed by worker/thread
        # self.speed_factor = INITIAL_SPEED_FACTOR # Not needed here
        # self.follow_camera_active = True # Using global
        self.previous_pos = self.data['pos'][0]
        self.current_display_time = self.data['t'][0]
        self.frame_update_counter = 0

        # Trajectory storage
        self.max_trail_points = None
        if USE_LIMITED_TRAIL and self.data['avg_dt'] > 0:
            self.max_trail_points = int(TRAIL_DURATION_SECONDS / self.data['avg_dt']) + 5
            self.trajectory_points_deque = collections.deque(maxlen=self.max_trail_points)
            print(f"Trajectory trail limited to approx {TRAIL_DURATION_SECONDS}s.")
        else:
            self.trajectory_points_list = []

        # Worker Thread Setup
        self.thread = None
        self.worker = None

        self.setWindowTitle("Rocket Animation"); self.setGeometry(100, 100, 1400, 950) # Reduced height slightly
        central_widget = QWidget(); self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Plotter
        self.plotter = pvqt.BackgroundPlotter(show=False, window_size=(1200,800))
        self.plotter.set_background(BACKGROUND_COLOR); self.plotter.camera.up = (0.0, 0.0, 1.0)
        if DISABLE_ANTI_ALIASING: self.plotter.disable_anti_aliasing(); print("Anti-aliasing disabled.")
        plotter_frame = QFrame(); plotter_layout = QVBoxLayout(plotter_frame)
        plotter_layout.addWidget(self.plotter.interactor); main_layout.addWidget(plotter_frame, 1)

        # Controls - Simplified
        controls_widget = QWidget()
        # Set fixed height for controls area
        controls_widget.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        controls_layout = QHBoxLayout(controls_widget) # Use QHBoxLayout now
        main_layout.addWidget(controls_widget)

        # Play/Pause Button
        self.play_pause_button = QPushButton("Play");
        self.play_pause_button.setCheckable(True) # Use checkable state
        self.play_pause_button.clicked.connect(self.toggle_play_pause)
        controls_layout.addWidget(self.play_pause_button)

        # Time Label (optional, but useful)
        self.time_label = QLabel(f"Time: 0.00 / {self.total_duration:.2f} s")
        controls_layout.addWidget(self.time_label)
        controls_layout.addStretch() # Push button/label left

        # Setup Initial Scene
        self.setup_actors(); self.update_scene(self.data['t'][0])

        print("\nAnimator Ready."); print(f"- Target Display FPS: {TARGET_DISPLAY_FPS}")


    def setup_actors(self):
        """Create and add all static and dynamic actors to the plotter."""
        # (Same actor setup as before)
        rocket_mesh=pv.Cone(center=(0,0,ROCKET_HEIGHT/2.),direction=(0,0,1),height=ROCKET_HEIGHT,radius=ROCKET_RADIUS,resolution=ROCKET_RESOLUTION)
        self.rocket_actor=self.plotter.add_mesh(rocket_mesh,color=ROCKET_COLOR,smooth_shading=True)
        nozzle_h=ROCKET_HEIGHT*NOZZLE_HEIGHT_FACTOR;nozzle_r=ROCKET_RADIUS*NOZZLE_RADIUS_FACTOR
        nozzle_mesh=pv.Cylinder(center=(0,0,-nozzle_h/2.),direction=(0,0,1),radius=nozzle_r,height=nozzle_h,resolution=NOZZLE_RESOLUTION,capping=True)
        self.nozzle_actor=self.plotter.add_mesh(nozzle_mesh,color=NOZZLE_COLOR)
        thrust_mesh=pv.Arrow(start=(0,0,0),direction=(0,0,-1),scale=THRUST_LENGTH_SCALE,tip_length=0.3,tip_radius=0.15,shaft_radius=0.05)
        self.thrust_actor=self.plotter.add_mesh(thrust_mesh,color=THRUST_COLOR)
        self.trajectory_mesh=pv.PolyData();self.trajectory_actor=self.plotter.add_mesh(self.trajectory_mesh,color=TRAJECTORY_COLOR,line_width=3)
        try:# Earth
            if hasattr(pv,'load_globe_texture'):earth_texture=pv.load_globe_texture();earth_mesh=pv.Sphere(radius=EARTH_VISUAL_RADIUS,center=EARTH_CENTER,theta_resolution=EARTH_THETA_RES,phi_resolution=EARTH_PHI_RES);self.plotter.add_mesh(earth_mesh,texture=earth_texture);print("Added textured Earth sphere.")
            else:raise AttributeError("load_globe_texture not found")
        except Exception as e:print(f"Could not load Earth texture: {e}");earth_mesh=pv.Sphere(radius=EARTH_VISUAL_RADIUS,center=EARTH_CENTER,theta_resolution=EARTH_THETA_RES,phi_resolution=EARTH_PHI_RES);self.plotter.add_mesh(earth_mesh,color='deepskyblue')
        self.plotter.show_grid(bounds=GRID_BOUNDS,color=GRID_COLOR,location='origin');self.plotter.add_axes(line_width=5,labels_off=False)
        initial_pos=self.data['pos'][0];self.plotter.camera.focal_point=initial_pos;self.plotter.camera.position=initial_pos+INITIAL_CAM_OFFSET
        self.previous_pos=initial_pos.copy()

    # SLOT connected to worker's update_needed signal
    def update_scene(self, target_time):
        """Updates actor positions and orientations for a given target time."""
        if ENABLE_PERFORMANCE_TIMING: t_start_update = time.perf_counter()

        target_time=np.clip(target_time,self.data['t'][0],self.data['t'][-1]);
        self.current_display_time=target_time # Update internal time tracking

        pos=np.zeros(3);rocket_angle_rad=0.;nozzle_dev_rad=0.

        # Interpolation
        idx1=np.searchsorted(self.data['t'],target_time,side='right')
        idx0=max(0,idx1-1);idx1=min(self.num_data_points-1,idx1)
        current_data_index = idx0

        if idx0==idx1:pos=self.data['pos'][idx0];rocket_angle_rad=self.data['theta'][idx0];nozzle_dev_rad=self.data['nozzle_angle'][idx0]
        else:
            t0,t1=self.data['t'][idx0],self.data['t'][idx1];pos0,pos1=self.data['pos'][idx0],self.data['pos'][idx1]
            theta0,theta1=self.data['theta'][idx0],self.data['theta'][idx1];nozzle0,nozzle1=self.data['nozzle_angle'][idx0],self.data['nozzle_angle'][idx1]
            frac=(target_time-t0)/(t1-t0)if(t1-t0)>1e-9 else 0.;frac=np.clip(frac,0.,1.)
            pos=pos0+frac*(pos1-pos0);rocket_angle_rad=theta0+frac*(theta1-theta0);nozzle_dev_rad=nozzle0+frac*(nozzle1-nozzle0)

        # Orientations
        rocket_rot_y_rad=math.pi/2.-rocket_angle_rad;rocket_rot_y_deg=math.degrees(rocket_rot_y_rad)
        nozzle_rot_y_deg=rocket_rot_y_deg-math.degrees(nozzle_dev_rad);thrust_rot_y_deg=nozzle_rot_y_deg

        # Update Actors
        self.rocket_actor.position=pos;self.rocket_actor.orientation=(0.,rocket_rot_y_deg,0.)
        self.nozzle_actor.position=pos;self.nozzle_actor.orientation=(0.,nozzle_rot_y_deg,0.)
        self.thrust_actor.position=pos;self.thrust_actor.orientation=(0.,thrust_rot_y_deg,0.)

        # Update Trajectory Storage & Mesh (Every Frame)
        current_traj_points=None
        if self.max_trail_points is not None: # Deque
            last_deque_index=-1
            if self.trajectory_points_deque:
                 try:
                    last_pt_in_deque=self.trajectory_points_deque[-1];matches=np.where(np.all(self.data['pos']==last_pt_in_deque,axis=1))[0]
                    if len(matches)>0:last_deque_index=matches[-1]
                 except IndexError:pass
            if current_data_index>last_deque_index: self.trajectory_points_deque.append(self.data['pos'][current_data_index])
            current_traj_points=list(self.trajectory_points_deque)
        else: # Unlimited List
             if current_data_index>=len(self.trajectory_points_list): self.trajectory_points_list.append(self.data['pos'][current_data_index])
             current_traj_points=self.trajectory_points_list

        # Update PolyData
        if current_traj_points and len(current_traj_points)>=2:
            points_array=np.array(current_traj_points);self.trajectory_mesh.points=points_array
            try:self.trajectory_mesh.lines=pv.lines_from_points(points_array).lines
            except ValueError:self.trajectory_mesh.lines=np.array([],dtype=int)
            self.trajectory_mesh.Modified()
        else:
             self.trajectory_mesh.points=np.empty((0,3),dtype=float);self.trajectory_mesh.lines=np.array([],dtype=int)
             self.trajectory_mesh.Modified()

        # Update Camera
        if follow_camera_active: # Use fixed global setting
            self.plotter.camera.focal_point=pos.tolist();delta_pos=pos-self.previous_pos
            current_cam_pos=np.array(self.plotter.camera.position);new_cam_pos=current_cam_pos+delta_pos
            self.plotter.camera.position=new_cam_pos.tolist()

        # Update Time Label Only
        self.time_label.setText(f"Time: {target_time:.2f} / {self.total_duration:.2f} s")
        self.previous_pos=pos.copy()

        # Performance Timing
        if ENABLE_PERFORMANCE_TIMING:
            t_end_update=time.perf_counter();update_duration_ms=(t_end_update-t_start_update)*1000
            self.frame_update_counter+=1
            if self.frame_update_counter%PRINT_TIMING_FREQUENCY==0:print(f"Time {target_time:.2f}s: update_scene took {update_duration_ms:.2f} ms")

    # --- GUI Interaction Slots ---
    def toggle_play_pause(self):
        if self.play_pause_button.isChecked(): # User wants to play
            if self.thread is None or not self.thread.isRunning():
                 self.start_worker() # Start new thread
            else:
                 self.worker.resume() # Resume existing worker
            self.play_pause_button.setText("Pause")
        else: # User wants to pause
            if self.worker:
                 self.worker.pause()
            self.play_pause_button.setText("Play")

    def start_worker(self):
        print("Starting animation worker thread...")
        self.thread = QThread()
        # Pass frame duration, not speed factor (worker uses global ANIMATION_SPEED_FACTOR)
        self.worker = AnimationWorker(self.data, FRAME_DURATION_SEC)
        self.worker.moveToThread(self.thread)

        # Connect signals (only update_needed and finished now)
        self.worker.update_needed.connect(self.update_scene)
        # self.worker.progress_update.connect(self.update_slider_position) # REMOVED
        self.worker.finished.connect(self.on_worker_finished)
        self.thread.started.connect(self.worker.run)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)

        # Restart logic
        if self.current_display_time >= self.data['t'][-1]: self.current_display_time = self.data['t'][0]

        self.thread.start()

    def on_worker_finished(self):
        print("Worker finished signal received.")
        self.play_pause_button.setChecked(False) # Ensure button is back to 'Play' state
        self.play_pause_button.setText("Play")
        # self.scrub_slider.setEnabled(True) # No scrub slider anymore
        self.thread = None; self.worker = None # Clear references

    # REMOVED update_slider_position, scrub handlers, set_speed, toggle_camera_mode

    def closeEvent(self, event):
        print("Closing animator...")
        if self.worker: self.worker.stop()
        if self.thread:
            self.thread.quit()
            if not self.thread.wait(1000): print("Warning: Worker thread did not exit.")
        if hasattr(self,'plotter')and self.plotter:self.plotter.close()
        event.accept()

# --- Script Entry Point ---
if __name__=="__main__":
    import os
    if not os.path.exists(CSV_FILENAME):
        print(f"Creating dummy data file: {CSV_FILENAME}")
        # Dummy data generation...
        time_points=np.linspace(0,12,300);x_points=np.interp(time_points,[0,12],[0,22.5]);y_points=np.zeros_like(time_points);z_points=np.interp(time_points,[0,12],[0,1440]);theta_points=np.interp(time_points,[0,12],[1.57079,0.85]);nozzle_points=0.03*np.sin(time_points*2*np.pi/3)
        with open(CSV_FILENAME,'w') as f:f.write("t,X,Y,Z,theta,nozzleAngle\n");[f.write(f"{t:.4f},{x:.4f},{y:.4f},{z:.4f},{th:.5f},{nz:.5f}\n")for t,x,y,z,th,nz in zip(time_points,x_points,y_points,z_points,theta_points,nozzle_points)]

    rocket_data=load_data(CSV_FILENAME)
    app=QApplication(sys.argv);window=RocketAnimatorApp(rocket_data);window.show()
    if pyqt_version==6:sys.exit(app.exec())
    else:sys.exit(app.exec_())