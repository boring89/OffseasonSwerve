package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.lib.Swerve.SwerveSetpoint;
import frc.robot.lib.Swerve.SwerveSetpointGenerator;

public class drive extends SubsystemBase {

    private final driveIO io;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private final Field2d field = new Field2d();

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    public drive(driveIO io) {

        this.io = io;

        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.io.getRotation2d(),
                this.io.getModulePositions(),
                new Pose2d());

        this.swerveSetpointGenerator = new SwerveSetpointGenerator(
                this.kinematics,
                DriveConstants.moduleLocations);

        io.zeroHeading();
        io.resetEncoders();
    }

    public void runVelocity(ChassisSpeeds speeds) {

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, io.getRotation2d());

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);

        SwerveModuleState[] SetPointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                0.02);

        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SetPointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        this.io.setModuleStates(setpointStates);
    }

    public void stop() {
        this.io.stopModules();
    }

    public void zeroHeading() {
        this.io.zeroHeading();
    }

    @Override
    public void periodic() {
        poseEstimator.update(
                this.io.getRotation2d(),
                this.io.getModulePositions());

        field.setRobotPose(this.poseEstimator.getEstimatedPosition());

        SmartDashboard.putData("field", field);

        SmartDashboard.putNumber("heading", io.getHeading());
    }

}
