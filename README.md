# autonomous_drone
Using a DJI Ryze Tello drone to perform autonomous flight actions. I'm using Python for testing but the productional code is C++

Steps:

  (1) Setup an API for the backend (-> UDP interface: Sending flight commands, receiving sensor signals and video stream)
  
  (2) Synchronize the API for usage in simulation environment Microsoft Airsim (https://github.com/Microsoft/AirSim)
  
  (3) Using OpenCV for perception tasks (object detection, semantic segmentation)
  
  (4) Perform some basic autonomous flight maneuvers in the simulation environment
  
  (5) Transfer to real-world environment
