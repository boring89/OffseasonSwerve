package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.lib.Swerve.Module.ModuleIO;
import frc.robot.lib.Swerve.Module.ModuleIO.ModuleIOInputs;
import frc.robot.lib.Swerve.Module.ModuleIOSpark;

public class driveIOHardware implements driveIO {

    private final ModuleIO[] modules = new ModuleIO[4];
    private final ModuleIOInputs[] moduleInputs = new ModuleIOInputs[4];
    private final AHRS gyro;

    public driveIOHardware() {
        for (int i = 0; i < 4; i++) {
            this.moduleInputs[i] = new ModuleIOInputs();
            this.modules[i] = new ModuleIOSpark(i, DriveConstants.kModuleReversed[i]);
            this.modules[i].updateInputs(moduleInputs[i]);
        }
        this.gyro = new AHRS(NavXComType.kMXP_SPI);
    }

    @Override
    public void updateInputs(driveIOInputs inputs) {

        for (int i = 0; i < 4; i++) {
            this.modules[i].updateInputs(moduleInputs[i]);
        }

        inputs.gyroheading = getRotation2d();
        inputs.gyroTurnRate = getTurnRate();
        inputs.moduleStates = getModuleStates();
        inputs.modulePositions = getModulePositions();
    }

    @Override
    public void zeroHeading() {
        this.gyro.reset();
    }

    public double getHeading() {
        return -this.gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getHeading());
    }

    public double getTurnRate() {
        return gyro.getRate();
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = moduleInputs[i].moduleState;
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = moduleInputs[i].modulePosition;
        }
        return positions;
    }

    @Override
    public void swerveOutput(SwerveModuleState[] state) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                state, DriveConstants.kMaxSpeedMeterPerSecond);
        for (int i = 0; i < 4; i++) {
            modules[i].setModuleState(state[i]);
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].stop();
        }
    }

    @Override
    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            modules[i].resetEncoders();
        }
    }
}