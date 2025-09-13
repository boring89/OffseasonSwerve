package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
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

        xOutput = Math.abs(xOutput) > OIConstants.kDeadband ? xOutput : 0.0;
        yOutput = Math.abs(yOutput) > OIConstants.kDeadband ? yOutput : 0.0;
        rotOutput = Math.abs(rotOutput) > OIConstants.kDeadband ? rotOutput : 0.0;

        if (xOutput == 0 && yOutput == 0 && rotOutput == 0) {
            this.drive.stop();
        } else {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOutput, yOutput, rotOutput);

            drive.runVelocity(chassisSpeeds);
        }

    }
}
