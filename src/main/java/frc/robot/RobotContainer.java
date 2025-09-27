// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.driveCmd;
import frc.robot.subsystems.control.Controller;
import frc.robot.subsystems.drive.drive;
import frc.robot.subsystems.drive.driveIOHardware;

public class RobotContainer {

  private final Controller driver = new Controller();

  private final driveIOHardware driveIO = new driveIOHardware();
  private final drive drive = new drive(driveIO);

  public RobotContainer() {
    this.drive.setDefaultCommand(
      new driveCmd(
        drive, 
        () -> -driver.getLeftY(), 
        () -> -driver.getLeftX(), 
        () -> driver.getRightX()));

    configureBindings();
  }

  private void configureBindings() {
    driver.zeroHeading().onTrue(
      new InstantCommand(() -> this.drive.zeroHeading())
    );
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
