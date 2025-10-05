package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.DriveConstants;
import frc.robot.lib.Swerve.SwerveSetpoint;
import frc.robot.lib.Swerve.SwerveSetpointGenerator;
import frc.robot.subsystems.drive.driveIO.driveIOInputs;

public class drive extends SubsystemBase {

    private final driveIO io;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private double[] PhotonPose = new double[3];

    private Rotation2d yaw;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });


    private final driveIOInputs inputs = new driveIOInputs();

    public drive(driveIO io) {

        this.io = io;

        Pose2d initialPose = new Pose2d(0, 0, inputs.gyroheading);

        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                inputs.gyroheading,
                inputs.modulePositions,
                initialPose);

        this.swerveSetpointGenerator = new SwerveSetpointGenerator(
                this.kinematics,
                DriveConstants.moduleLocations);

        this.field = new Field2d();

        io.zeroHeading();
        io.resetEncoders();
    }

    public void runVelocity(ChassisSpeeds speeds) {

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, inputs.gyroheading);

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

        this.io.swerveOutput(setpointStates);
    }

    public void autoAlign(double[] speeds) {
        this.runVelocity(new ChassisSpeeds(speeds[0], speeds[1], speeds[2]));
    }

    public void stop() {
        this.io.stop();
    }

    public void zeroHeading() {
        this.io.zeroHeading();
    }

    public Command resetHeading() {
        return runOnce(() -> this.zeroHeading());

    }

    public void addVisionMeasurement(Pose2d pose) {
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }

    @Override
    public void periodic() {

        poseEstimator.update(
                inputs.gyroheading,
                inputs.modulePositions);

        double[] pose = {
                poseEstimator.getEstimatedPosition().getX(),
                poseEstimator.getEstimatedPosition().getY(),
                Math.toRadians(inputs.gyroheading.getDegrees())
        };

        SmartDashboard.putNumberArray("PhotonPose", PhotonPose);

        SmartDashboard.putNumberArray("Pose", pose);

        SmartDashboard.putNumber("heading", inputs.gyroheading.getDegrees());

        SmartDashboard.putNumber("turnRate", inputs.gyroTurnRate);
    }

}
