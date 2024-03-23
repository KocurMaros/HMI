# Kobuki Controller

## Table of contents

 - [Description](#description)
 - [Application structure](#application-structure)
 - [Control panel](#control-panel)
 - [Settings](#settings)
 - [Body control](#body-controlling)

### Description

This application is used to control a mobile Kobuki robot on local network. There are
mul tiple ways how to view information from the robot and how to control it. The prefered
way how to contro l the Kobuki robot is using the keyboard. It is either by using **WSDA**
or **arrows**. To **stop** the robo t press **R**, to initiate the **emergency stop**
press **ESC**.

### Application structure:

In the top menu bar we can see two sections. First are the Settings and the seco nd one is
this Help message. The main window consist of the main screen where all the data are
displayed and a control panel.

### Control panel:

  1. **Led:** Indicator for the current state (Connected, Disconnected, Emergency Stop).
  This led also offeres the current IP address of the robot.
  2. **Combo box:** IP addresses. You can choose here the IP address of your Kobuki robot.

  3. **Connect / Disconnect:** Button to initiate or terminate the connection.
  4. **Use camera:** Switch between Lidar and robot's camera mode.
  5. **Robot's position:** X, Y and Rotation of the robot.
  6. **Emergency stop:** This button will push the robot into emergency stop state.

In this state you can only switch between the lidar and camera view. Movement is prohibited.

### Settings:

  1. **Colorscheme:** You can choose between 3 colorschemes. Default, coffee, mocha.
  2. **Control buttons:** Show the control buttons in the app.
  3. **Left-handed mode:** Switch the app into left handed mode.

### Body controlling:

For the controll is this application using left and right hand. Where left hand controls
the velocity of th robot and the right hand controls the rotation of th e robot robot. For
the robot to stop the hands need to be in 45 degrees ( corssed ). Upward mov ement means
increase in the control unit.

Thank you for using this controller.
