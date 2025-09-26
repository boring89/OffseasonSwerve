package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.constants.Constants.OIConstants;
import frc.robot.subsystems.drive.drive;

public class driveCmd extends Command {

    private final drive drive;

    private final Supplier<Double> xSpd, ySpd, rotSpd;

    public driveCmd(drive drive,
            Supplier<Double> xSpeedSupplier,
            Supplier<Double> ySpeedSupplier,
            Supplier<Double> rotSpeedSupplier) {

        this.drive = drive;

        this.xSpd = xSpeedSupplier;
        this.ySpd = ySpeedSupplier;
        this.rotSpd = rotSpeedSupplier;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xOutput = xSpd.get();
        double yOutput = ySpd.get();
        double rotOutput = rotSpd.get();

        xOutput = applyDeadband(xOutput) * DriveConstants.kMaxSpeedMeterPerSecond;
        yOutput = applyDeadband(yOutput) * DriveConstants.kMaxSpeedMeterPerSecond;
        rotOutput = applyDeadband(rotOutput) * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

        if (xOutput == 0 && yOutput == 0 && rotOutput == 0) {
            this.drive.stop();
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);

            drive.runVelocity(chassisSpeeds);
        }

    }

    public double applyDeadband(double Speed) {
        return Math.abs(Speed) > OIConstants.kDeadband ? Speed : 0.0;
    }
}
