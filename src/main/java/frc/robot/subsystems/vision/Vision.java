package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.TagConstants;
import frc.robot.subsystems.drive.drive;

public class Vision extends SubsystemBase {

    private final drive drive;

    private final NetworkTable left_Limelight, right_Limelight;
    private final PhotonCamera left_Photon, Right_Photon;

    private final Transform3d L_CAM_TO_ROBOT = new Transform3d(0.20979456, 0.13607890, 0.15952705,
            new Rotation3d(0.0, 0.0, Math.toRadians(30)));
    private final Transform3d R_CAM_TO_ROBOT = new Transform3d(-0.20979456, 0.13607890, 0.15952705,
            new Rotation3d(0.0, 0.0, Math.toRadians(-30)));

    private final PhotonPoseEstimator left_PoseEstimator, right_PoseEstimator;

    private Pose2d FLPhotonPose, FRPhotonPose;

    public Vision(drive drive) {

        this.drive = drive;

        left_Limelight = NetworkTableInstance.getDefault().getTable("limelight-left");
        right_Limelight = NetworkTableInstance.getDefault().getTable("limelight-right");

        left_Photon = new PhotonCamera("LeftOV");
        Right_Photon = new PhotonCamera("RightOV");

        left_PoseEstimator = new PhotonPoseEstimator(
                TagConstants.FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new Transform3d());

        right_PoseEstimator = new PhotonPoseEstimator(
                TagConstants.FIELD_LAYOUT,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                R_CAM_TO_ROBOT);

        left_PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        right_PoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void getLimelightPose() {
        if (left_Limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]) != null) {
            double[] FLPose = left_Limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            this.drive.addVisionMeasurement(new Pose2d(FLPose[0], FLPose[1], Rotation2d.fromDegrees(FLPose[5])));
        }
        if (right_Limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]) != null) {
            double[] FRPose = right_Limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            this.drive.addVisionMeasurement(new Pose2d(FRPose[0], FRPose[1], Rotation2d.fromDegrees(FRPose[5])));
        }
    }

    public void getPhotonPose() {

        var leftResult = left_Photon.getLatestResult();
        var rightResult = Right_Photon.getLatestResult();

        FLPhotonPose = (leftResult != null)
                ? left_PoseEstimator.update(leftResult)
                        .map(est -> est.estimatedPose.toPose2d())
                        .orElse(new Pose2d())
                : new Pose2d();

        FRPhotonPose = (rightResult != null)
                ? right_PoseEstimator.update(rightResult)
                        .map(est -> est.estimatedPose.toPose2d())
                        .orElse(new Pose2d())
                : new Pose2d();

        SmartDashboard.putNumberArray("PhotonTargetIDs", leftResult.getTargets().stream()
                .mapToDouble(t -> t.getFiducialId())
                .toArray());

    }

    public Pose2d getLeftPhotonPose() {
        return FLPhotonPose;
    }

    public Pose2d getRightPhotonPose() {
        return FRPhotonPose;
    }

    @Override
    public void periodic() {
        getLimelightPose();
        getPhotonPose();
    }
}
