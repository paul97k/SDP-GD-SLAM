import numpy as np
import matplotlib.pyplot as plt
import os
import sys

import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt
from scipy.ndimage import uniform_filter1d  # Alternative: np.convolve



# Ensure working directory and import dependencies
os.chdir("/home/paul/colcon_ws/src")
sys.path.append("/home/paul/colcon_ws/src/orca4/orca_bringup/scripts/")

from class_icp import Align2D


class ICP_SLAM:
    def __init__(self, file_path: str, file_dvl_path: str, skip_i: int = 5, window_size: int = 20, num_bins: int = 500):
        """
        Initialize the SonarICPProcessor.
        
        Parameters:
            file_path (str): Path to the sonar scan .npz file.
            skip_i (int): Number of scans to skip per iteration.
            window_size (int): Moving average window size.
        """
        
        self.file_path = file_path
        self.file_dvl_path = file_dvl_path
        self.skip_i = skip_i
        self.window_size = window_size
        self.num_bins = num_bins

        # Load sonar scan data 
        self.angles, self.ranges, self.timestamps_sonar = self.load_sonar_data()

        # Initialize position tracking
        self.pos = np.zeros(3)
        self.position_list = self.pos[np.newaxis, :]  # Store position history
        self.position_list = np.vstack([self.position_list, self.pos])

        self.position_list_dvl = self.position_list
        # Store scan results
        self.scan_list = []
        self.scan_list_withDVL = []
        self.icp_aligned_scans = []


        # Load dvl scan data and calculate position  
        self.vx, self.vy,  self.vz, self.timestamps_dvl = self.load_dvl_data()
        self.x_pos, self.y_pos, self.z_pos = self.compute_position_from_velocity()
    def get_scan_list(self):
        """Returns the processed scan list with DVL alignment."""
        return self.scan_list
    

    def set_icp_aligned_scans(self, x):
        self.icp_aligned_scans = x
    

    def get_position_list(self):
        return self.position_list
    
    def get_position_list_dvl(self):
        return self.position_list_dvl

    def get_scan_list_withDVL(self):
        return self.scan_list_withDVL
        
    def load_sonar_data(self):
        """Loads sonar scan data from an .npz file."""
        print(f"Loading sonar scan data from {self.file_path}...")
        loaded_data = np.load(self.file_path)
        return loaded_data["angles"], loaded_data["ranges"], loaded_data["timestamps"]
    
    def load_dvl_data(self):
        """Loads sonar scan data from an .npz file."""
        print(f"Loading sonar scan data from {self.file_dvl_path}...")
        loaded_data = np.load(self.file_dvl_path)
        return loaded_data["vx"], loaded_data["vy"], loaded_data["vz"], loaded_data["time_stamps"]
    
    def integrate_velocities(self):
        """Integrate velocities to compute positions using the Trapezoidal Rule."""
        self.x_pos, self.y_pos, self.z_pos = self.compute_position_from_velocity()

    def compute_position_from_velocity(self):

        dt = np.gradient(self.timestamps_dvl)  # Compute time differences

        x_pos = np.cumsum(self.vx * dt)  # Integrating vx → X position
        y_pos = np.cumsum(self.vy * dt)  # Integrating vy → Y position
        z_pos = np.cumsum(self.vz * dt)  # Integrating vz → Z position

        return x_pos, y_pos, z_pos
    

    def process_scans(self):
        """Iterates over sonar scans, applies ICP, and tracks position shifts."""
        num_iterations = len(self.timestamps_sonar) - 1 - self.skip_i
        optimal_lag_it = 0

        z1 = self.ranges[0] * np.cos(self.angles[0])
        t1 = self.ranges[0] * np.sin(self.angles[0])


        for i in range(1, num_iterations, self.skip_i):
            z2 = self.ranges[i +  self.skip_i] * np.cos(self.angles[i +  self.skip_i])
            t2 = self.ranges[i +  self.skip_i] * np.sin(self.angles[i +  self.skip_i])

            z1 = self.moving_average(z1, self.window_size)
            z2 = self.moving_average(z2, self.window_size)

            # t1 = np.linspace(-3, 3, len(z1))
            # t2 = np.linspace(-3, 3, len(z2))


            optimal_lag, _ = self.compute_optimal_lag(z1, z2, t1, t2, False)
            shifted_t = t2+optimal_lag
            # Update position
            self.pos[0] += optimal_lag
            self.position_list = np.vstack([self.position_list, self.pos])

            # Store scans
            # self.scan_list.append(np.column_stack((t1 + optimal_lag_it, z1, np.ones_like(z1), np.ones_like(z1))))
            # self.scan_list.append(np.column_stack((shifted_t + optimal_lag_it, z2, np.ones_like(z2), np.ones_like(z2))))

            # scan = np.column_stack((np.array(t1+optimal_lag_it) , 
            #                         np.array(z1) , 
            #                         np.ones_like(z1) , 
            #                         np.ones_like(z1)))  # Homogeneous coordinates
            # self.scan_list.append(scan)

            scan = np.column_stack((np.array(shifted_t+optimal_lag_it) , 
                                    np.array(z2) , 
                                    np.ones_like(z2) , 
                                    np.ones_like(z2)))  # Homogeneous coordinates
            self.scan_list.append(scan)

            # stored_signals[i] = {
            #     "t1": t1 + optimal_lag_it,
            #     "t2": t2 + optimal_lag_it,
            #     "shifted_t": shifted_t + optimal_lag_it,
            #     "z1": z1,
            #     "z2": z2,
            #     "optimal_lag": optimal_lag
            # }
            z1 = z2
            t1 = t2

            optimal_lag_it += optimal_lag




    def process_scans_with_dvl(self):
        """Iterates over sonar scans, applies ICP, and tracks position shifts."""
        num_iterations = len(self.timestamps_sonar) - 1 
        # plt.figure(figsize=(10, 6))
        
        for i in range(1, num_iterations):
            z1 = self.ranges[i] * np.cos(self.angles[i])
            z1 = self.moving_average(z1, self.window_size)
            
            x1 = self.ranges[i] * np.sin(self.angles[i])

            t_sonar = self.timestamps_sonar[i]
            t_idx_dvl = np.argmin(np.abs(self.timestamps_dvl - t_sonar))
            

            x = self.z_pos[t_idx_dvl]
            y = self.x_pos[t_idx_dvl]
            z = self.y_pos[t_idx_dvl]
            self.position_list_dvl = np.vstack([self.position_list, [x,y,z]])

            # print("t_sonar:", t_sonar, " | Nearest DVL Index:", t_idx_dvl, " | x:", x)


            scan = np.column_stack((np.array(x1-x) , 
                                    np.array(z1-y) , 
                                    np.ones_like(z1) , 
                                    np.ones_like(z1)))  # Homogeneous coordinates
            self.scan_list_withDVL.append(scan)
        #       plt.clf()
            
        #     plt.scatter((x1-x), z1, s=2, label=f"Scan {i}" if i == 0 else None)
        #     plt.xlabel("X Position (m)")
        #     plt.ylabel("Z Position (m)")
        #     plt.title("Sonar Scans with DVL Correction")
        #     plt.legend(["Sonar Scan"])
        #     plt.grid()
        #     plt.axis("equal")  # Ensure equal aspect ratio
        #     plt.pause(0.1)
        # plt.show()






    def convert_to_numpy(self):
        """Convert scan lists to NumPy arrays."""
        if len(self.scan_list) < 2:
            print("Not enough scans for ICP alignment.")
            return

        self.scan_list = [np.array(scan) for scan in self.scan_list]
        self.icp_aligned_scans = [self.scan_list[0][:, :3]]  # Store only (x, y, z)

       
    def align_scans_icp(self, scan_list, pos):
        """Aligns the scans using ICP."""
        print(f"Extracted {len(scan_list)} scans from ROS bag.")

        init_T = np.eye(3)  # Initial transformation guess
        icp_aligned_scans = []


        plotBool = True
        plotBool = False
        if plotBool: 
            fig, axs = plt.subplots(1, 2, figsize=(10, 5))  # Create subplots once

        Transform = np.identity(3)
        first_scan = scan_list[0][:, :3]


        skipScan = True
        if skipScan:
            for i in range(1, len(scan_list) - 1):
                first_scan = scan_list[i][:, :3]
                icp_aligned_scans.append(first_scan[:, :3])
        else:


            for i in range(1, len(scan_list) - 1):
            # for i in range(1, round(0.50*len(scan_list)) - 1):
                # print(f"Aligning scan {i} with scan {i-1} using ICP...")
                # first_scan = scan_list[i][:, :3]
                second_scan = scan_list[i + 1][:, :3] @ Transform.T


                len_first = len(first_scan)
                len_second = len(second_scan)
                
                cutPerc = 0.4

                cut_ids = round(cutPerc*len_first)
                cut_ide= round((1-cutPerc)*len_second)


                first_scan_cut = first_scan[:cut_ide,:]
                second_scan_cut = second_scan[cut_ids:,:]
                

                icp = Align2D(second_scan_cut,first_scan_cut, Transform) #source_points, target_points
                transformed_scan , _ , T= icp.transform  # Output transformed scan
                # self.position_list[i,:] = self.position_list[i,:]@T.T
                # print(T[0,2]-Transform[0,2], T[1,2]-Transform[1,2])
                # print(" ")
                Transform[0,2] = T[0,2]
                Transform[1,2] = T[1,2]


                transformed_scan = second_scan@ Transform.T
                transformed_scan_cut = second_scan_cut@ Transform.T

                if plotBool:
                    axs[0].cla()
                    axs[1].cla()
                    axs[0].plot(first_scan[:, 0], first_scan[:, 1], 'og', label="target/first scan")
                    axs[0].plot(first_scan_cut[:, 0], first_scan_cut[:, 1], 'or', label="target/first scan")
                    axs[0].plot(second_scan[:, 0], second_scan[:, 1], 'og', label="source/second scan") 
                    axs[0].plot(second_scan_cut[:, 0], second_scan_cut[:, 1], 'ob', label="source/second scan") 
                    axs[0].set_title("Before initial guess")
                    axs[0].legend()

                    axs[1].plot(first_scan_cut[:, 0], first_scan_cut[:, 1], 'or', label="target/first scan")
                    axs[1].plot(transformed_scan_cut[:, 0], transformed_scan_cut[:, 1], 'ob', label="source/second scan")
                    axs[1].set_title("After initial guess")
                    axs[1].legend()

                    plt.pause(0.01)
                first_scan = transformed_scan[:, :3]
                # Store the icp_aligned scan
                icp_aligned_scans.append(transformed_scan[:, :3])
            if plotBool: 
                plt.show()

        
        print("ICP alignment completed.")
        self.merge_scans(scan_list, icp_aligned_scans, pos)
    
        return icp_aligned_scans

    def merge_scans(self, scan_list, icp_aligned_scans, pos):
        """Merges original and icp_aligned scans."""
        # Extract X, Y, Z values from original scans
        original_x = np.concatenate([-scan[:, 0] for scan in scan_list])
        original_y = np.concatenate([-scan[:, 1] for scan in scan_list])
        original_z = np.concatenate([scan[:, 2] for scan in scan_list])

        # Extract X, Y, Z values from icp_aligned scans (ICP result)
        icp_aligned_x = np.concatenate([-scan[:, 0] for scan in icp_aligned_scans])
        icp_aligned_y = np.concatenate([-scan[:, 1] for scan in icp_aligned_scans])
        icp_aligned_z = np.concatenate([scan[:, 2] for scan in icp_aligned_scans])

        # Merge scans using binning
        merged_x, merged_y, merged_z = self.merge_scans_func(original_x, original_y, original_z, self.num_bins)
        merged_x_a, merged_y_a, merged_z_a = self.merge_scans_func(icp_aligned_x, icp_aligned_y, icp_aligned_z, self.num_bins)
        self.plot_results(original_x, original_y, merged_x, icp_aligned_x, icp_aligned_y, merged_y,  merged_x_a, merged_y_a, pos)


    def plot_results(self, original_x, original_y, merged_x, icp_aligned_x, icp_aligned_y, merged_y,  merged_x_a, merged_y_a , pos):
        """Plots the icp_aligned scans and merged results."""
        plt.figure(figsize=(8, 6))
        plt.scatter(original_x, original_y, s=1, alpha=0.2, label="Original Scans")
        plt.scatter(icp_aligned_x, icp_aligned_y, s=1, alpha=0.2, label="ICP Scans")


        plt.scatter(merged_x, merged_y, s=1, color='red', label="Merged Signal (Average)")
        plt.plot(-pos[:, 0], pos[:, 1], color='black', linewidth=2, label="Position")
        plt.plot(merged_x_a, merged_y_a, color='blue', linewidth=2, label="Merged ICP Signal (Average)")

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Merged Sonar Scans (X-Y Projection)")
        plt.legend()
        plt.grid()
        # plt.xlim([-100,100])
        # plt.ylim([-100,100])
        # plt.show()



    def plot_dvl_pos(self):
        # Plot DVL velocities
        plt.figure(figsize=(10, 5))
        plt.plot(self.timestamps_dvl, self.vx, label="Vx")
        plt.plot(self.timestamps_dvl, self.vy, label="Vy")
        plt.plot(self.timestamps_dvl, self.vz, label="Vz")

        plt.xlabel("Sample Index (or Time)")
        plt.ylabel("Velocity (m/s)")
        plt.legend()
        plt.title("DVL Velocities from Saved Data")
        plt.grid()


        # Plot the integrated positions
        plt.figure(figsize=(10, 5))
        plt.plot(self.timestamps_dvl, self.x_pos, label="X Position")
        plt.plot(self.timestamps_dvl, self.y_pos, label="Y Position")
        plt.plot(self.timestamps_dvl, self.z_pos, label="Z Position")

        plt.xlabel("Sample Index (or Time)")
        plt.ylabel("Position (m)")
        plt.legend()
        plt.title("Integrated Position from DVL Velocities")
        plt.grid()



    def plot_sonar_scans(self, step=1):
        """
        Plots the sonar scans with DVL alignment.
        :param step: Step size to skip scans for better visualization.
        """
        plt.figure(figsize=(10, 6))
        
        for i in range(0, len(self.scan_list), step):
            scan = self.scan_list[i]
            x_scan = -scan[:, 0]  # Extract X coordinates
            z_scan = -scan[:, 1]  # Extract Z coordinates

            plt.scatter(x_scan, z_scan, s=2, label=f"Scan {i}" if i == 0 else None)  # Plot scan points
        
        plt.xlabel("X Position (m)")
        plt.ylabel("Z Position (m)")
        plt.title("Aligned Sonar Scans using ICP")
        plt.legend(["Sonar Scan"])
        plt.grid()

    def print_positions(self):
        """Prints the stored positions."""
        for pos in self.position_list:
            print(pos)


    def moving_average(self, signal, window_size=5):
            return uniform_filter1d(signal, size=window_size)  # Efficient smoothing
    
    def cut_signal(self, cut_time_s: float, cut_time_e: float, signal1: np.ndarray, signal2: np.ndarray):
        # Generate time array based on signal1 length
        t = np.linspace(-3, 3, len(signal1))
        # Find the closest indices corresponding to cut_time_s and cut_time_e
        cut_ids = np.argmin(np.abs(t - cut_time_s))
        cut_ide = np.argmin(np.abs(t - cut_time_e))

        # Cut signal2 within the specified time range
        signal2_cut = signal2[cut_ids:cut_ide]

        # Generate the new time array for the cut signal
        t2 = np.linspace(cut_time_s, cut_time_e, len(signal2_cut))

        return t2, signal2_cut		

    def compute_optimal_lag(self, signal1: np.ndarray, signal2: np.ndarray, t1: np.ndarray, t2: np.ndarray, plot: bool = True):


        len_first = len(signal1)
        len_second = len(signal2)

        cutPerc = 0.3

        cut_ide= round((1-cutPerc)*len_first)
        cut_ids = round(cutPerc*len_second)


        signal1 = signal1[:cut_ide]
        signal2 = signal2[cut_ids:]

        # t1 = t1[:cut_ide]
        # t2 = t2[cut_ids:]


        # signal1 = signal1[cut_ids:]
        # signal2 = signal2[:cut_ide]

        # cut_time_s = -1.5
        # cut_time_e = -cut_time_s

        # t2, signal2 = self.cut_signal(cut_time_s, cut_time_e, signal1, signal2)

        # Compute cross-correlation
        corr_values = np.correlate(signal1, signal2, mode="full")

        # Compute lag values in time
        lags = np.linspace(-t1[-1], t1[-1], len(t2)+ len(t1)-1)


        # Find optimal lag (corresponding to max absolute correlation)
        optimal_lag = lags[np.argmax(np.abs(corr_values))]
        optimal_lag = -np.abs(optimal_lag)+0.2

        # optimal_lag = -0.25
        
        # Shift the time axis by the optimal lag
        shifted_t = t2-optimal_lag
        # shifted_t = t2-0.60731 
        
        if plot:
        # Plot results
            fig, axes = plt.subplots(2, 1, figsize=(8, 8))

            # Plot original and shifted signals
            axes[0].plot(t1, signal1, label="Signal 1", linewidth=2)
            axes[0].plot(t2, signal2, linestyle="dashed", label="Original Signal 2", linewidth=2)
            axes[0].plot(shifted_t, signal2, linestyle="dashdot", label=f"Signal 2 with Shifted Time (lag={optimal_lag:.5f})", linewidth=2)
            axes[0].set_xlabel("Time (s)")
            axes[0].set_ylabel("Amplitude")
            axes[0].set_title("Signal Alignment using Cross-Correlation")
            axes[0].legend()
            axes[0].grid()

            # Plot cross-correlation
            axes[1].plot(lags, corr_values, label="Cross-Correlation", color='tab:red')
            axes[1].axvline(optimal_lag, color='black', linestyle='--', label=f"Optimal Lag = {optimal_lag:.5f}")
            axes[1].set_xlabel("Lag (s)")
            axes[1].set_ylabel("Cross-Correlation")
            axes[1].set_title("Cross-Correlation of Signals")
            axes[1].legend()
            axes[1].grid()

            plt.tight_layout()
            plt.show()

        return optimal_lag, shifted_t




    def merge_scans_func(self, original_x: np.ndarray, original_y: np.ndarray, original_z: np.ndarray, num_bins: int = 500):
        """
        Merge overlapping scans by binning X values and averaging corresponding Y and Z values.

        Parameters:
            original_x (np.ndarray): X coordinates of the original scans.
            original_y (np.ndarray): Y coordinates of the original scans.
            original_z (np.ndarray): Z coordinates of the original scans.
            num_bins (int): Number of bins for the X axis.

        Returns:
            merged_x (np.ndarray): Binned X values.
            merged_y (np.ndarray): Averaged Y values corresponding to merged X.
            merged_z (np.ndarray): Averaged Z values corresponding to merged X.
        """
        # Define evenly spaced X values between min and max of original_x
        merged_x = np.linspace(min(original_x), max(original_x), num_bins)

        # Create lists to store averaged Y and Z values
        merged_y = np.full(num_bins - 1, np.nan)  # Initialize with NaN
        merged_z = np.full(num_bins - 1, np.nan)  # Initialize with NaN

        # Loop through each bin and compute the average Y and Z
        for i in range(len(merged_x) - 1):
            # Find indices of original points that fall within the current bin
            indices = (original_x >= merged_x[i]) & (original_x < merged_x[i + 1])

            if np.any(indices):  # If there are points in this bin
                merged_y[i] = np.mean(original_y[indices])
                merged_z[i] = np.mean(original_z[indices])

        # Optionally interpolate missing values (NaNs)
        merged_y = np.nan_to_num(merged_y, nan=np.nanmean(original_y))
        merged_z = np.nan_to_num(merged_z, nan=np.nanmean(original_z))

        return merged_x[:-1], merged_y, merged_z  # Return bin centers
