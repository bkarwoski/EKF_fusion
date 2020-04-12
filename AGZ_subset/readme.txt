*-------------------------------------------------------------------------------------------------------*
Please note: The image data in the folder ./AGZ/MAV Images/ and ./AGZ/MAV Images Calib/ are property of the 
authors Andras L. Majdik, Yves Albers-Schoenberg and Davide Scaramuzza. They can be used for academic research without any limitations. 
The images in the folder ./AGZ/Street View Images/ are property of Google Inc. please refer to www.google.com/streetview for further details. 
*-------------------------------------------------------------------------------------------------------*

./AGZ/Log Files/BarometricPressure.csv
This file contains data from the onoard barometric presure sensor:
1: Timestemp, 2: Pressure, 3: Altitude, 4: Temperature

./AGZ/Log Files/GroundTruthAGL.csv
1: imgid (id of the MAV image), 2: x_gt (ground truth camera position x), 3: y_gt (ground truth camera position y), 4: z_gt (ground truth camera position z), 5: omega_gt (degrees, ground truth camera orientation yaw), 6: phi_gt (degrees, ground truth camera orientation pith), 7: kappa_gt (degrees, ground truth camera orientation roll), 8: x_gps (GPS camera position x), 9: y_gps (GPS camera position y), 10: z_gps (GPS camera position z)
All values regarding positions are in the WGS 84 / UTM zone 32N coordinate system use plotPath.m to visualize the data in matlab

./AGZ/Log Files/GroundTruthAGM
1: Timestemp, 2: imgid (id of the MAV image), 3: svid_1 (id of the closest Google Street View image), 4: svid_2 (id of the second closest Google Street View image), 5: svid_3 (id of the third closest Google Street View image)
The distance was computed by using the raw GPS tags.

./AGZ/Log Files/OnbordGPS.csv
This file contains the GPS data from the MAV onboard GPS receiver:
1: Timestemp, 2: imgid (id of the MAV image), 3: lat (Latitude in 1E7 degrees), 4: lon (Longitude in 1E7 degrees), 5: alt (Altitude in 1E3 meters (millimeters) above MSL), 
6: s_variance_m_s (speed accuracy estimate m/s ), 7: c_variance_rad (course accuracy estimate rad), 8: fix_type (0-1: no fix, 2: 2D fix, 3: 3D fix), 
9: eph_m (GPS HDOP horizontal dilution of position in m ), 10: epv_m (GPS VDOP horizontal dilution of position in m ), 11: vel_n_m_s (GPS ground speed in m/s ), 
12: vel_e_m_s (GPS ground speed in m/s), 13: vel_d_m_s (GPS ground speed in m/s ), 14: num_sat (Number of satellites visible.)
more details: https://home.hibu.no/AtekStudenter1212/doxygen/bb_handler/structvehicle__gps__position__s.html

./AGZ/Log Files/OnboardPose.csv
This file contains the raw swnsor data and the estimated onboard pose by PIXHAWK autopilot http://pixhawk.org/modules/pixhawk:
1: Timpstemp, 2: Omega_x, 3: Omega_y, 4:Omega_z, 5: Accel_x, 6: Accel_y, 7: Accel_z, 8: Vel_x, 9: Vel_y, 10: Vel_z, 11: AccBias_x, 12: AccBias_y, 13: AccBias_z, 14: Azimuth, 15: Attitude_w, 16: Attitude_x, 17: Attitude_y, 18: Attitude_z, 19: Height, 20: Altitude, 21: veh_pitch, 22: Tether_angle, 23: Tether_angle_dot, 24: Tether_force, 25: GPS_on

./AGZ/Log Files/RawAccel.csv
1:Timpstemp, 2: Error_count, 3: x, 4: y, 5: z, 6: temperature, 7: range_rad_s, 8: scaling, 9: x_raw, 10: y_raw, 11: z_raw, 12: temperature_raw

./AGZ/Log Files/RawGyro.csv
1: Timpstemp, 2: Error_count, 3: x, 4: y, 5: z, 6: temperature, 7: range_rad_s, 8: scaling, 9: x_raw, 10: y_raw, 11: z_raw, 12: temperature_raw


./AGZ/Log Files/StreetViewGPS.csv
This file contains the GPS data (geotagging) for every database Street View image, the format is according to the Google Street View API:
1: latitude, 2:longitude, 3: yaw_degree, 4: tilt_yaw_degree, 5:tilt_pitch degree, 6: auxiliary variable

./AGZ/MAV Images/
This folder contains 81'169 images recorded by an MAV in the city of Zurich, Switzerland.

./AGZ/MAV Images Calib/
This folder contains 30 images recorded by an MAV, these are used to compute the intrinsic camera parameters. 

./AGZ/Street View Images/
This folder contains 113 Street View Images corresponding to the area recorded by the MAV.

./AGZ/calibration_data.npz
These are the internal camera parameters computed using the images from './AGZ/MAV Images Calib/'

./AGZ/loadGroundTruthAGL.m
This script is used by plotPath.m to load the data into Matlab.

./AGZ/plotPath.m
Use this script to visualise the path in Matlab.

./AGZ/write_ros_bag.py
This file write the data to a ros bag file. To execute simply type in a terminal >> python write_ros_bag.py