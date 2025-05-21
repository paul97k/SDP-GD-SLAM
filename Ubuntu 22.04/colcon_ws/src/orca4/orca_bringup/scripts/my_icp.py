import os
import sys
from class_ICP_SLAM import ICP_SLAM
import matplotlib.pyplot as plt
import numpy as np

os.chdir("/home/paul/colcon_ws/src")
sys.path.append("/home/paul/colcon_ws/src/orca4/orca_bringup/scripts/")


# if __name__ == "__main__":
file_path = "orca4/orca_bringup/scripts/data npz/sonar_scan_data1.npz"
file_dvl_path = "orca4/orca_bringup/scripts/data npz/vdl_data.npz"

processor = ICP_SLAM(file_path, file_dvl_path, skip_i=5, window_size=20, num_bins=200)

processor.process_scans()       # Process sonar scans
processor.plot_sonar_scans()       # Process sonar scans
aligned_scans= processor.align_scans_icp(processor.get_scan_list(), processor.get_position_list())     # Align using ICP


processor.process_scans_with_dvl()  
# # processor.plot_dvl_pos()

DVLScanList = processor.get_scan_list_withDVL()
filtered_list = [DVLScanList[i] for i in range(1, len(DVLScanList), 10)]

aligned_scans= processor.align_scans_icp(filtered_list, processor.get_position_list_dvl())     # Align using ICP

plt.show()