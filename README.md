# head-in-space
This repository holds the code being developed for the head-in-space aspect of my PhD project. It is very much a work in progress.

The main work is in the Scripts folder. If you create a new, default Unity project then add in these scripts, along with the colours provided in the 'Materials' folder and install TextMesh Pro (used to mark the directions on the rotating cuboid) it should work as intended.

The corresponding Arduino code to setup the earables headset and communicate with Unity is also here. Use the magnetometer calibration codes to find the hard and soft magnetometer offsets, then add these in to the arduino-unity-madgwick.ino code to avoid issues with misalignment.

See also a short demo video (unity-madgwick_Trim.mp4) of the cuboid rotating to follow the IMU.
