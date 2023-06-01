# (Day 1) Transparency and haptic feedback

The M1-AnkleMotus is a commercial ankle rehabilitation robot developed by Fourier Intelligence (Singapore). We made some hardware modifications to bypass the default controller, allowing us to access the motor drive and torque sensor directly. This open source software stack (CORC) is used to control the ankle robot.

This README file gives instructions for manually tuning a controller for the M1 robot, with the goal of achieving transparent control (i.e., near zero interaction torque) and haptic rendering (e.g., assistance-as-needed) during typical ankle movements. We also provide details for visualizing interaction torques between the user and the robot, as well as visual feedback for an ankle exercise.

## Check git branch and build project

1. Open a terminal (ctrl + alt + T)
2. Make sure you are on the demo/ssnr2023 branch for both CANOpenRobotController and multi_robot_interaction

```bash
cd ~/catkin_ws/src
cd CANOpenRobotController
git checkout demo/ssnr2023

cd ~/catkin_ws/src
cd multi_robot_interaction
git checkout demo/ssnr2023
```
3. Build the CORC project and source the workspace
```bash
cd ~/catkin_ws
catkin build CORC
source devel/setup.bash
```

## Initialize CAN communication

1. Connect one of the M1 devices (m1_x,  m1_y or m1_z) to the operating computer via CAN USB cable
2. From a terminal, initialize CAN communication for a single device on can0 (enter password: *123456* when prompted)
```bash
cd ~/catkin_ws/src/CANOpenRobotController/script
sudo ./initCAN0.sh
```
3. Return to the project workspace
```bash
cd ~/catkin_ws
```
## Tune the transparent controller

1. In [m1_params.yaml](../../../config/m1_params.yaml) file, make sure *config_flag* is set to true in order to enable sliders for adjusting PID gains
2. Run ROS program for controller tuning from the same terminal window where the workspace was sourced; specify the robot_name parameter for your group’s device (m1_x, m1_y or m1_z)
```bash
roslaunch CORC m1_real.launch robot_name:=m1_x

roslaunch CORC m1_real.launch robot_name:=m1_y

roslaunch CORC m1_real.launch robot_name:=m1_z
```
3. On the `/m1_<robot_name>` panel of **Dynamic Reconfigure**, set *controller_mode* to *zero_calibration* and wait until calibration is complete (i.e., foot pedal is parallel to the ground); because the M1 robot has an relative encoder, calibration is needed to determine the home position (0 deg, CW+)
   * If the pedal is not parallel to the ground after calibrating, there is an error; set *controller_mode* to *zero_torque*, then *zero_calibration* again to retry
4. Set *controller_mode* to *virtual_spring*
   * **Note:** with the PID and feedforward gains all set to 0, *virtual_spring* mode will operate like *zero_torque* mode (large amount of resistance due to no compensation for friction/gravity)
5. Strap into the device while seated in a chair
6. Adjust the sliders on the `/m1_<robot_name>` panel of **Dynamic Reconfigure** to change the amount of feedforward compensation (*ff_ratio*) and PID gains (*kp*, *ki* and *kd*)
   * A suggestion, but not a requirement, is to start by tuning the *ff_ratio*, which controls the amount of compensation for the weight of the pedal and friction of the shaft/motor; then sequentially tune the *kp*, *ki* and *kd* gains, in this order
   * **Note:** make small, incremental changes to each slider individually before testing; large gains can cause instability/oscillations
7. Move your ankle back and forth as you tune each gain of the controller
   * The goal is to feel like you can move your ankle freely (no resistance from the device) while avoiding oscillations or overcompensation caused by the controller
8. For more precise tuning and visualization, start the **Multiplot** display by pressing the play icon at the top right
   * The top subplot will display your instantaneous ankle angle
   * The bottom subplot will display the measured and desired interaction torques (desired torque is 0 for transparent control)
9. To display a target trajectory for an example ankle exercise, set the experimental conditions in the `/multi_robot_interaction` panel of **Dynamic Reconfigure**
   * Set *trajectory_mode* to *multi_sine*
   * Set *experimental_cond* to *disable*
   * Set *interaction_mode* to *spring_interaction_tracking* (starts the experiment)
   * Adjust the *f_1*, *f_2* and *f_3* sliders to change the frequency of the multi-sine; adjust *amp* to change the amplitude range
10. Once you have decided on a set of gains amongst your group, make a note of the tuned values, then press ctrl + C on the terminal window to end the program

## Evaluate controller peformance during ankle exercise
1. As a group, pick 2-3 sets of feedforward/PID gains you would like to evaluate across members as well as the number of trials/target frequencies during ankle tracking (*spring_interaction_tracking*)
      * Controller performance can be evaluated using interaction torque tracking errors
      * Data from each M1 robot (interaction torque; desired and actual trajectories etc.) are continuously logged from when roslaunch is called, to when the program is ended (ctrl + C)
2. Run ROS program from the terminal where the workspace was sourced; specify the robot_name parameter for your group’s device (m1_x, m1_y or m1_z)
```bash
roslaunch CORC m1_real.launch robot_name:=m1_x

roslaunch CORC m1_real.launch robot_name:=m1_y

roslaunch CORC m1_real.launch robot_name:=m1_z
```
3. On the `/m1_<robot_name>` panel of **Dynamic Reconfigure**, set *controller_mode* to *zero_calibration* and wait until calibration is complete
4. Adjust the sliders on the `/m1_<robot_name>` panel of **Dynamic Reconfigure** to a set of gains you would like to test
5. Set *controller_mode* to *virtual_spring*
6. Strap into the device while seated in a chair
7. Start the **Multiplot** display by pressing the play icon at the top right 
8. Perform trials of example ankle exercise after configuring the experimental conditions in the `/multi_robot_interaction` panel of **Dynamic Reconfigure**
   * Set *trajectory_mode* to *multi_sine*
   * Set *experimental_cond* to *disable*
   * Set *interaction_mode* to *spring_interaction_tracking* (starts the experiment)
9. Press ctrl + C on the terminal window to end the program after desired number of trials are performed
10. For each group member, repeat steps 2 through 9 for each set of gains
11. Navigate to the log folder within ~\.ros\spdlogs and find the folder for your robot (m1_x, m1_y or m1_z)
12. Transfer the .csv files associated with one or more previous runs (check timestamps) to a separate laptop
13. Using software of your choice (MATLAB, R studio, python), analyze the data from multiple runs to observe how interaction torque measurements change depending on the controller gains you've set
   * If using MATLAB, see [`m1_post_process.m`](../../../matlab/m1_post_process.m); this function will load a .csv file, segment it into trials (if the *interaction_mode* was set to *spring_interaction_tracking*), and compute the root-mean-square error of the interaction torque measurements and tracking errors; it is suggested that you run this function in a loop to collect and analyze interaction torque errors across all group members
   * If not using MATLAB, key variables to look at in the .csv file would be the time (column name: time), mode (column name: mode), actual joint angle (column name: JointPositions_1) and desired joint angle (column name: MM1_DesiredJointPositions_1)

## Test haptic feedback
1. Run ROS program for haptic feedback from the terminal where the workspace was sourced; specify the robot_name parameter for your group’s device (m1_x, m1_y or m1_z)
```bash
roslaunch CORC m1_real.launch robot_name:=m1_x

roslaunch CORC m1_real.launch robot_name:=m1_y

roslaunch CORC m1_real.launch robot_name:=m1_z
```
2. On the `/m1_<robot_name>` panel of **Dynamic Reconfigure**, set *controller_mode* to *zero_calibration* and wait until calibration is complete
3. Adjust the sliders on the `/m1_<robot_name>` panel of **Dynamic Reconfigure** to the set of gains you previously configured
4. Set *controller_mode* to *virtual_spring*
5. Strap into the device while seated in a chair
6. Start the **Multiplot** display by pressing the play icon at the top right
7. To render a haptic virtual spring between your pedal and a fixed angle, set the *interaction_mode* to *spring_interaction*
   * Adjust the *k_interaction* to change the stiffness parameter of the spring
9. To display a target trajectory for an example ankle exercise with haptic feedback, set the experimental conditions in the `/multi_robot_interaction` panel of **Dynamic Reconfigure**
   * Set *trajectory_mode* to *multi_sine*
   * Set *experimental_cond* to *enable* (spring always on) or *alternate* (switch spring on and off each trial)
   * Set *interaction_mode* to *spring_interaction_tracking* (starts the experiment)
10. When you are finished, press ctrl + C on the terminal window to end the program

## Customize haptic feedback
1. Open your C++ editor (CLion or Visual Studio) and navigate to MultiRobotInteraction.cpp within ~\catkin_ws\src\multi_robot_interaction\src
2. Read through the code in the section defining the *spring_interaction_tracking* mode (Lines 157-166)
```cpp
traj = amp_*(0.33*sin(f_1_*2*M_PI*time) + 0.33*sin(f_2_*2*M_PI*time) + 0.33*sin(f_3_*2*M_PI*time));
// set interaction torque and multi-sine target angle
for(int robot = 0; robot<numberOfRobots_; robot++){
   interactionEffortCommandMatrix_(dof, robot) =
           connected_flag * k_interaction_ * (bias - jointPositionMatrix_(dof, robot));
   jointPositionCommandMatrix_(dof, robot) = traj + bias;
}
```
3. Adapt this section to implement a different type of haptic feedback discussed in the presentation (e.g., assist-as-needed, error augmentation)
   * **Note:** in the current version, this section computes the difference between the current ankle angle (jointPositionMatrix_(dof,robot)) and a constant angle (bias), then multiplies this difference by the GUI configued spring constant (k_interaction_) to calculate the desired effort command (interactionEffortCommandMatrix_(dof, robot))
4. After modifying the file, rebuild the CORC project and source the workspace
```bash
cd ~/catkin_ws
catkin build CORC
source devel/setup.bash
```
5. Follow the steps in the **Test haptic feedback** section to try out the new haptic feedback mode you’ve created
