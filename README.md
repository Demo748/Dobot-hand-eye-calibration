# Dobot-hand-eye-calibration
Sensors and control project 4 hand-eye coordination/calibration

Uses Dobot Magician, Realsense D435 RGBD camera, ROS, MATLAB

**Demonstration.m**
The main file that runs the project demonstration all in one. Performs calibration and moves Dobot to different poses to take calibration images, generates camera parameters, which are used for visual servoing to make the robot follow the camera. Captured images are saved to /demo/patternEffector or /demo/cameraEffector depending on which setup is being performed. The pattern_effector variable can be changed to switch setups. Poses/joint configurations for calibration can be modified

**DobotMagician.m** class file helper used to initialise the ros connection and contains helper functions to get joint values, publish end effector poses or target joint values, etc.

**Main.m** assuming calibration has already been performed/camera parameters generated and present in the workspace, this can be run to test visual servoing of Dobot following of the camera

Other matlab scripts were all used in testing/experimenting and built upon to create the final demonstration.m file. E.g. testing movement of dobot, testing image capture of camera, generating calibration parameters off of preloaded images, calibration portion of the code, main loop code, etc.
