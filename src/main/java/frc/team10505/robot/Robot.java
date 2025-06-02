// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team10505.robot;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //TODO initialize robot container
  //AND create autonomous command

  public Robot() {}

  @Override
  public void robotPeriodic() {
    //TODO add stuff (you guys best know what goes here by now)
    //walter is not the supreme anything
  }

  @Override
  public void autonomousInit() {
    //TODO schedule autonomous command (referense season code if need be)
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    //TODO cancel autonomous command (referense season code if need be)
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
