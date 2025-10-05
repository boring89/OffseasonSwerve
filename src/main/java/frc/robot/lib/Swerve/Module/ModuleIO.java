package frc.robot.lib.Swerve.Module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class ModuleIO {

    public static class ModuleIOInputs {
        public double drivePosition = 0.0;
        public double turnPosition = 0.0;

        public double driveVelocity = 0.0;
        public double turnVelocity = 0.0;

        public double absoluteAngle = 0.0;

        public SwerveModulePosition modulePosition = new SwerveModulePosition();
        public SwerveModuleState moduleState = new SwerveModuleState();
    }

    public abstract void updateInputs(ModuleIOInputs inputs);

    public abstract void setModuleState(SwerveModuleState state);

    public abstract void resetEncoders();
    public abstract void stop();

    public static SwerveModuleState optimizeState(SwerveModuleState desiredState, double turnPosition) {
        desiredState.optimize(Rotation2d.fromRadians(turnPosition));
        return desiredState;
    }
}
