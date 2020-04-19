# autonomous_drone
Using a DJI Ryze Tello drone to perform autonomous flight actions. I'm using Python for testing but the productional code is C++

All code is written by me, unless otherwise indicated. I try to write clean code regarding my coding style and architecture if possible.

Steps:

    (1) Setting up an API for the backend (-> UDP interface: Sending flight commands, receiving sensor signals and video stream)
  
    (2) Setting up a decoder and converter to receive a bit map from the raw video data for further CV usage
  
    (3) Implementing a visual SLAM algorithm with the drone's mono camera, OpenCV and the drone's IMU data
    
    (4) ... will be defined after step 3 :)
