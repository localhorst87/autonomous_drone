# autonomous_drone
Using a DJI Ryze Tello drone to perform autonomous flight actions. I'm using Python for testing but the productional code is C++

All code is written by me, unless otherwise indicated. I try to write clean code regarding my coding style and architecture if possible.

Steps:

    (1) Setting up an API for the backend (-> UDP interface: Sending flight commands, receiving sensor signals and video stream)
  
    (2) Setting up a decoder and converter to receive a bit map from the raw video data for further CV usage
  
    (3) Synchronize the API for usage in simulation environment Microsoft Airsim (https://github.com/Microsoft/AirSim)
  
    (4) Using OpenCV for perception tasks (object detection, semantic segmentation)
  
    (5) Perform some basic autonomous flight maneuvers in the simulation environment
  
    (6) Transfer to real-world environment
