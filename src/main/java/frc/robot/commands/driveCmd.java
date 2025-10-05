package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.control.Controller;
import frc.robot.subsystems.drive.drive;

public class driveCmd extends Command {

    private final drive drive;

    private final Controller driver, co_driver;

    private double[] driverInputs, co_driverInputs, SwerveOutputs;

    public driveCmd(drive drive, Controller driver, Controller co_driver) {

        this.drive = drive;

        this.driver = driver;
        this.co_driver = co_driver;

        this.driverInputs = new double[3];
        this.co_driverInputs = new double[3];

        this.SwerveOutputs = new double[3];

        addRequirements(drive);
    }

    @Override
    public void execute() {

        this.driverInputs = new double[] {
                applyDeadband(-driver.getLeftY()) * DriveConstants.kMaxSpeedMeterPerSecond,
                applyDeadband(-driver.getLeftX()) * DriveConstants.kMaxSpeedMeterPerSecond,
                applyDeadband(driver.getRightX()) * DriveConstants.kMaxAngularSpeedRadiansPerSecond
        };
        this.co_driverInputs = new double[] {
                applyDeadband(-co_driver.getLeftY()) * DriveConstants.kMaxSpeedMeterPerSecond,
                applyDeadband(-co_driver.getLeftX()) * DriveConstants.kMaxSpeedMeterPerSecond,
                applyDeadband(co_driver.getRightX()) * DriveConstants.kMaxAngularSpeedRadiansPerSecond
        };



        if (co_driverInputs[0] == 0 && co_driverInputs[1] == 0 && co_driverInputs[2] == 0) {
            SwerveOutputs = driverInputs;
        } else {
            SwerveOutputs = co_driverInputs;
        }

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(SwerveOutputs[0], SwerveOutputs[1], SwerveOutputs[2]);

        drive.runVelocity(chassisSpeeds);
    }

    public double applyDeadband(double Speed) {
        return Math.abs(Speed) > OIConstants.kDeadband ? Speed : 0.0;
    }
}
