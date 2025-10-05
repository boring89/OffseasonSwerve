
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.driveCmd;
import frc.robot.subsystems.control.Controller;
import frc.robot.subsystems.drive.drive;
import frc.robot.subsystems.drive.driveIOHardware;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {

  private final Controller driver = new Controller(0);
  private final Controller co_driver = new Controller(1);



  private final driveIOHardware driveIO = new driveIOHardware();
  private final drive drive = new drive(driveIO);

  private final Vision vision = new Vision(drive);

  public RobotContainer() {
    this.drive.setDefaultCommand(
      new driveCmd(drive, driver, co_driver));

    configureBindings();
  }

  private void configureBindings() {
    driver.zeroHeading().onTrue(drive.resetHeading());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
