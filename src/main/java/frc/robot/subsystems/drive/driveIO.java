package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface driveIO {

    public static class driveIOInputs {

        public Rotation2d gyroheading = new Rotation2d();
        public double gyroTurnRate = 0.0;
        public SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    }

    void updateInputs(driveIOInputs inputs);

    void swerveOutput(SwerveModuleState[] states);

    void zeroHeading();
    void resetEncoders();

    void stop();
}
