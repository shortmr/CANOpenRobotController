# (Day 1) Transparency and haptic feedback 

## Overview
The M1-AnkleMotus is a commercial ankle rehabilitation robot developed by Fourier Intelligence (Singapore). We made some hardware modifications to bypass the default controller, allowing us to access the motor drive and torque sensor directly. This open source software stack (CORC) is used to control the ankle robot.

This README file gives instructions for manually tuning a controller for the M1 robot, with the goal of achieving transparent control (i.e., near zero interaction torque) and haptic rendering (e.g., assistance-as-needed) during typical ankle movements. We also provide details for visualizing interaction torques between the user and the robot, as well as visual feedback for an ankle exercise.

## Check git branch and build project

First, initialize the CAN device (No need to repeat this step, unless you disconnect the CAN adapter):

For only robot A:
```bash
cd script
./initCAN0CAN1.sh
```
For only robot B:
```bash
cd script
./initCAN2CAN4.sh
```

For robot A and B:
```bash
cd script
./initCAN0124.sh
```

The adapter should start blinking after this command

![Figure](images/can_blink.gif)


### Ground Reaction Force (Treadmill or Pressure Pad)
The developed controller uses ground reaction forces to have a continuous gate state changing from 0 to 1. This allows having continous
interaction force estimation and forward model during the complete gait cycle (see the manuscript for details).
A sensorized treadmill or our custom pressure sensor pad can be used to have grf measurements. Note that currently, only robot B
have the pressure pad implementation.

For the details of the fsr sensor and calibration process, see [here](FSRPad.md).

#### Split-belt Treadmill
There are 4 Force sensors under each belt. These sensors measure force in all directions. We only use the vertical and
anterior/posterior forces for our sagittal plane analyses. Note that AP readings of the right belt are defective; therefore, in practice, only the left AP force is used.

The force sensors output voltage values in the range of [-5, 5]V. These outputs are linearly converted into [0 2.5]V range
 with a circuit before inputting into the Arduino near the window. There is a ROS node running in the Arduino that publishes the
 analog reading that it reads. Arduino code is [here](../../arduino/publishTreadmillAI.ino).
 
 Follow these steps in order to access treadmill data
 
 * Connect the Arduino cable to your PC
 * Turn on the power supply BEFORE before connecting red, green and black cables
 * Make sure both ports at 5V and they are at independent states (buttons in the middle are NOT pressed)

![Figure](images/power_supply.jpg)
* Turn of the power supply
* Connect the red, black and green cables
* Turn on the power supply
* Press the reset treadmill switch that is on the cabinet.
* Connect the treadmill cable to the ribbon cable

![Figure](images/treadmill_cable.jpg)

* In a new terminal: `roscore` 
* In a different terminal: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200`  
(dont't forget to `source devel/setup.bash before`  running)
(Note you might need to change ttyACM0 number but if there is no arduino connected, it should be 0. 
To make sure, you can check the connection port on an Arduino IDE)
* Check if `treadmill_sensor` is being published by running `rostopic list` 

**(IMPORTANT!: When you are done, FIRST disconnect the treadmil cable BEFORE turning off the power supply.)**

#### Pressure pad
Connect the 9V batteries on the side. Do not forget to disconnect when you are done!

#### CORC
For robot A:
```bash
roslaunch CORC x2_real_A.launch
```
rqt_gui is disabled by default for robot A(not to have multiple guis when bultiple robots are used). If desired to use only A and
have a gui, gui argument can be set to true:
```bash
roslaunch CORC x2_real_A.launch gui:=true
```

For robot B:
```bash
roslaunch CORC x2_real_B.launch
```

To create haptic interaction between the robots
```bash
roslaunch multi_robot_interaction x2_dyad.launch
```

### GUI

#### Service Call
Service calls are located on the right side of the gui.

##### Homing

ExoMotus-X2 has relative encoders; therefore, a homing procedure is needed. 
By using the service caller on the right side of the gui, you can command the homing procedure. 
Select `/<robot_name>/start_homing` and press `Call`.
Hip and knee joints will move forward until hitting the joint limit one by one. This should be done when there is no user in the exoskeleton.

There is no need to do homing again even if you restart the program (unless you do not turn off the robot).

##### Calibrate IMU
IMU orientation estimation might deteriorate over time and needs to be calibrated. Calibration is especially needed if
the location of the exoskeleton changes after the last calibration. Calibrating after the start of each program is suggested.
Call the `/<robot_name>/calibrate_imu` service on the gui to start the calibration. You will see warning messages on the terminal until the
calibration has been completed. During the calibration, the subject should be asked to stay still.

##### Calibrate Treadmill
Treadmill calibration should be done everytime the program is restarted. It should be done when there is no one on the
treadmill. It should be done only once for a single run. Calibration is done by calling the `<robot_name>_calibrate_treadmill`
service.

##### Calibrate Force Sensor
Before the user dons the Exo, it is suggested to check the interaction torques (see visualization [HERE](RTVisualizationForDebugging.md)
 for details). If it is too far away from zero (more than 5-10 Nm), most probably homing or calibration has not been done.
 If it is slightly off from zero (~3Nm), calibration of the force sensors might be needed. For this, put the legs completely
  on vertical configuration (you can uncomment the print statements at the beginning of the `during()` function of the X2DemoState
  class to see the link angles - should be ~90 deg.) Then call `<robot_name>_calibrate_force_sensors`. It will take around 10 seconds for all sensors to be calibrated.
  You should see the interaction torques are moved toward zero on the rqt_multiplot.
  
  (**Possible improvement: Write a service call (or expand the current one) that moves the joints so that each leg is aligned with gravity.**)

#### Controller mode
This version have the following modes:

* 1 - `Zero torque`: Switches to torque mode and commands zero torque to all motors
* 2- Zero velocity`: Switches to velocity mode and commands zero velocity to all motors
* 3- `ff_model_compensation`: Commands only the calculated feedforward torque (gravity, Coriolis and friction compensation)
terms. Very low performance, won't be transparent at all.
* 8- `interpolated_dynamic_acc_loop`: Virtual mass control with constrained optimization to control interaction torques.
(zero or non-zero)
* 10- `system_id`: This mode commands different torque profiles with different amplitudes and frequencies. Recorded data
can be used to identify friction, gravity and inertial elements.

Note that the controllers mode are run only during the green button on the handle is pressed (except for zero vel mode).
If green button is not pressed, zero torques are sent to the motors. This is implemented as a safety feature.

(See more details in [X2DemoState](../../src/apps/X2DemoMachine/states/X2DemoState.cpp)).

#### Gait State
Gait state can be manually chosen from gui, or automatic mode can be selected. If tests are done with no user while X2 is hung,
it is suggested to select the flying mode. If the tests include the user and treadmill sensors or pressure pads working,
 automatic (0) should be selected.
 
 #### Main Parameters
 * `virtual_mass_<0-1>`: The virtual inertia of the backpack, left thigh, left shank, right thigh and right shank, respectively.
 The lower the virtual mass, the more transparent and less stable the system is.
 * `double_stance_multiplier`: larger virtual masses are used during the double stance at all links for better stability.
 The virutal masses are multiplied by this amount during the double stance phase
 * `use_treadmill`: enable this if sensorized treadmill is used for gait state detection and force estimation. If not selected
 it is assumed pressure pads are used.
 * `interpolate_force_estimation`: when treadmill is used horizontal forces are also used for interaction torque estimation.
 Another possible way to calculate interaction forces is to just interpolating the force estimation from left and right dyanmics
 estimations. Select this to enable this simplified option. Note that with pressure pad implementation automatically this is used
 because there is no horizontal force estimation.
 * `enable_rendering`: enable this to render spring damper at the joints. If the following parameter (`subscribe_rendering`)
  is not selected this will give desired interaction torque values due to the spring/damper proporties listed in Section 
  `F_Rendering`.
  * `subscribe_rendering`: Enable this for dyadic haptic interaction. The desired interaction torques will be subscribed
  from the `<robot_name>/desired_interaction_torque` topic.
  
  #### Enable
  If a joint is not selected, always zero torque will be given to the robot.
  
  #### Gait State Threshold
  Threshold values in Newton to trigger contact with the ground for left and right.
  
  #### Force Smoothing
  * `compensate friction`: keep it enabled for friction compensation
  * `smooth_friction`: if enabled, a smooth friction profile is used.
  * `friction_vel_thresh` limit velocity in deg/s for the viscous velocity
  * `force_cutOff`: Interaction forces are low-pass filtered with this cuf off frequency in Hz.
  
  #### Coriolis
  * `use_coriolis`: enable coriolis compensation
  * `coriolis_cut_off`: cut off frequency [Hz] of the calculated coriolis forces
  
  #### Rendering
  Properties of the rendered spring/damper parameters if `enable_rendering` is selected and `subscribe_rendering` is disabled.
  
  #### Acceleration
Cut-off frequencies of the accelerations are calculated through time derivatives of joint or backpack velocity readings.
Not used in real-time control (for now), only for visualization and logging purposes.

  #### Drive Limits
The limits used in the constrained optimization. All units are SI. `delta_torque_limit` is a constraint on the rate of the
motor torque solution in consecutive time stamps. It was initially used to test but increased significantly to make it useless.
(**Possible improvement: Just delete the usage of that variable everywhere.**)

## Code development
  
To grasp the overview of the CORC, you can refer [here](../3.Software).

For the details of our implementation refer to [this documentation](CodeStructure.md).

## Real time visualization for debugging
See [this documentation](RTVisualizationForDebugging.md).

## Post process
See [`analyze_vCombine.m`](../../matlab/PostProcess/analyse_vCombine.m) as an example. 

## Unity feedback
See [this repository](https://github.com/emekBaris/x2Unity). If you do not have access, email `baris.kucuktabak@u.northwestern.edu`.
