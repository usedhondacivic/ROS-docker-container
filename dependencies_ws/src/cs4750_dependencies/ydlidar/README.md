# YDLIDAR
Package for lidar sensor.

Parameters
=====================================================================
port (string, default: /dev/ydlidar)

    serial port name used in your system. 

baudrate (int, default: 115200)

    serial port baud rate. 

frame_id (string, default: laser_frame)

    frame ID for the device. 

low_exposure (low_exposure, default: false)

    indicated whether the LIDAR has low light power mode. 

angle_fixed (bool, default: true)

    indicated whether the driver needs do angle compensation. 

heartbeat (bool, default: false)

    indicated whether the LIDAR IS powered off. 

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution. 

angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data. 

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data. 

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data. 

range_max (double, default: 16.0)

    Max valid range (m) for LIDAR data. 

ignore_array (string, default: "")

    Set the current angle range value to zero. 

samp_rate (int, default: 4)

    the LIDAR sampling frequency.

frequency (double, default: 7)

    the LIDAR scanning frequency.




Upgrade Log
=====================================================================
2018-04-16 version:1.3.1

   1.Update SDK verison to 1.3.1

   2.Increase sampling frequency,scan frequency setting.

   3.Unified coordinate system.

   4.Repair X4,S4 LIDAR cannot be opened.

   5.Increased G4 G4C F4Pro LIDAR power-off protection.

   6.Increased S4B LIDAR low optical power setting.

   7.Fix the wait time for closing ros node.
   
   8.Compensate for each laser point timestamp.

   9.Unified profile, automatic correction lidar model.






