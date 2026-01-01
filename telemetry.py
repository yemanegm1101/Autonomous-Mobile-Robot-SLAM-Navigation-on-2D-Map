import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import multiprocessing
import numpy as np
import time

class TelemetryPlotter:
    def __init__(self, queue):
        self.queue = queue
        self.times = []
        self.velocities = []
        self.omegas = []
        self.heading_errors = []
        self.planning_status = [] # 1: Success, -1: Failure, 0: Idle
        self.segments = []  # List of (time, label)
        self.start_time = time.time()
        
        # Max points to show on graph
        self.max_points = 200 # Increased for better analysis

    def run(self):
        self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, figsize=(10, 10))
        self.fig.suptitle("Robot Telemetry Analysis & Comparison", fontsize=16)
        
        # Velocity Plot
        self.line_v, = self.ax1.plot([], [], 'b-', label='Linear Vel (m/s)', linewidth=1.5)
        self.ax1.set_ylabel('m/s')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)
        
        # Omega Plot
        self.line_w, = self.ax2.plot([], [], 'r-', label='Angular Vel (rad/s)', linewidth=1.5)
        self.ax2.set_ylabel('rad/s')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)
        
        # Heading Error Plot
        self.line_err, = self.ax3.plot([], [], 'g-', label='Heading Error (rad)', linewidth=1.5)
        self.ax3.set_ylabel('rad')
        self.ax3.legend(loc='upper right')
        self.ax3.grid(True, alpha=0.3)
        
        # Planning Status Plot
        self.line_status, = self.ax4.plot([], [], 'm-', label='Planning Status', linewidth=1.5)
        self.ax4.set_ylabel('Status')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_yticks([-1.0, 0, 1.0])
        self.ax4.set_yticklabels(['Error', 'Idle', 'Success'])
        self.ax4.legend(loc='upper right')
        self.ax4.grid(True, alpha=0.3)

        self.animation = FuncAnimation(self.fig, self.update, interval=100, cache_frame_data=False)
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

    def update(self, frame):
        # Pull everything from queue
        while not self.queue.empty():
            try:
                data = self.queue.get_nowait()
                if data == "STOP":
                    plt.close(self.fig)
                    return
                
                if isinstance(data, dict) and "SEGMENT" in data:
                    t = time.time() - self.start_time
                    label = data["SEGMENT"]
                    self.segments.append((t, label))
                    
                    # Add vertical lines to graphs
                    for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                        ax.axvline(x=t, color='k', linestyle='--', alpha=0.5)
                        ax.text(t, ax.get_ylim()[1], label, rotation=90, verticalalignment='top', fontsize=8)
                    continue

                v, w, err, status = data
                self.times.append(time.time() - self.start_time)
                self.velocities.append(v)
                self.omegas.append(w)
                self.heading_errors.append(err)
                self.planning_status.append(status)
                
                # Keep only last max_points for real-time smoothness, 
                # but maybe keep more for "Comparison"? 
                # Let's keep a bit more.
                if len(self.times) > self.max_points:
                    self.times.pop(0)
                    self.velocities.pop(0)
                    self.omegas.pop(0)
                    self.heading_errors.pop(0)
                    self.planning_status.pop(0)
                    
                    # Clean up old segments too? No, let's keep them if they are in view.
            except:
                break

        if self.times:
            # Update data
            self.line_v.set_data(self.times, self.velocities)
            self.line_w.set_data(self.times, self.omegas)
            self.line_err.set_data(self.times, self.heading_errors)
            
            self.line_status.set_data(self.times, self.planning_status)
            
            # Rescale axes
            for ax in [self.ax1, self.ax2, self.ax3, self.ax4]:
                ax.relim()
                ax.autoscale_view()
            
            # Keep X axis moving
            xmin = self.times[0]
            xmax = max(self.times[-1], xmin + 10) # 10s window minimum
            self.ax4.set_xlim(xmin, xmax)
            self.ax1.set_xlim(xmin, xmax)
            self.ax2.set_xlim(xmin, xmax)
            self.ax3.set_xlim(xmin, xmax)

        return self.line_v, self.line_w, self.line_err, self.line_status

def start_telemetry(queue):
    plotter = TelemetryPlotter(queue)
    plotter.run()
