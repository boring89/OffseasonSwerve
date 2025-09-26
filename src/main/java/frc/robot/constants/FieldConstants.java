package frc.robot.constants;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;


//from 2910's code

public class FieldConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT;

    static {
        FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    public static final double FIELD_HEIGHT = 8.0518;
    public static final double FIELD_LENGTH = 17.548249;

    // Tag IDs
    public static final int RED_LEFT_CORAL_STATION = 1;
    public static final int RED_RIGHT_CORAL_STATION = 2;
    public static final int RED_PROCESSOR = 3;
    public static final int RED_RIGHT_NET = 4;
    public static final int RED_LEFT_NET = 5;
    public static final int RED_REEF_LEFT_DRIVER_STATION = 6;
    public static final int RED_REEF_CENTER_DRIVER_STATION = 7;
    public static final int RED_REEF_RIGHT_DRIVER_STATION = 8;
    public static final int RED_REEF_RIGHT_BARGE = 9;
    public static final int RED_REEF_CENTER_BARGE = 10;
    public static final int RED_REEF_LEFT_BARGE = 11;
    public static final int BLUE_RIGHT_CORAL_STATION = 12;
    public static final int BLUE_LEFT_CORAL_STATION = 13;
    public static final int BLUE_LEFT_BARGE = 14;
    public static final int BLUE_RIGHT_BARGE = 15;
    public static final int BLUE_PROCESSOR = 16;
    public static final int BLUE_REEF_RIGHT_DRIVER_STATION = 17;
    public static final int BLUE_REEF_CENTER_DRIVER_STATION = 18;
    public static final int BLUE_REEF_LEFT_DRIVER_STATION = 19;
    public static final int BLUE_REEF_RIGHT_BARGE = 20;
    public static final int BLUE_REEF_CENTER_BARGE = 21;
    public static final int BLUE_REEF_LEFT_BARGE = 22;

    public static final Translation2d RED_MARK_1 = new Translation2d(16.329, 2.197);
    public static final Translation2d RED_MARK_2 = new Translation2d(16.329, 4.026);
    public static final Translation2d RED_MARK_3 = new Translation2d(16.329, 5.855);
    public static final Translation2d BLUE_MARK_1 = new Translation2d(1.219, 5.855);
    public static final Translation2d BLUE_MARK_2 = new Translation2d(1.219, 4.026);
    public static final Translation2d BLUE_MARK_3 = new Translation2d(1.219, 2.197);

    public static final Pose2d FAR_LEFT_STARTING_POSE_BLUE = new Pose2d(7.2, 7, Rotation2d.k180deg);
    public static final Pose2d FAR_LEFT_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - FAR_LEFT_STARTING_POSE_BLUE.getX(),
            FIELD_HEIGHT - FAR_LEFT_STARTING_POSE_BLUE.getY(),
            Rotation2d.kZero);
    public static final Pose2d FAR_RIGHT_STARTING_POSE_BLUE = new Pose2d(
            FAR_LEFT_STARTING_POSE_BLUE.getX(), FIELD_HEIGHT - FAR_LEFT_STARTING_POSE_BLUE.getY(), Rotation2d.k180deg);
    public static final Pose2d FAR_RIGHT_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - FAR_RIGHT_STARTING_POSE_BLUE.getX(),
            FIELD_HEIGHT - FAR_RIGHT_STARTING_POSE_BLUE.getY(),
            Rotation2d.kZero);

    public static final Pose2d LEFT_STARTING_POSE_BLUE = new Pose2d(7.2, 6.14, Rotation2d.fromDegrees(45.0));
    public static final Pose2d LEFT_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - LEFT_STARTING_POSE_BLUE.getX(),
            FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(-135.0)); // todo red angles are wrong
    public static final Pose2d RIGHT_STARTING_POSE_BLUE = new Pose2d(
            LEFT_STARTING_POSE_BLUE.getX(),
            FIELD_HEIGHT - LEFT_STARTING_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(-45.0));
    public static final Pose2d RIGHT_STARTING_POSE_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_STARTING_POSE_BLUE.getX(),
            FIELD_HEIGHT - RIGHT_STARTING_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(135.0)); // todo red angles are wrong

    public static final Pose2d LEFT_STATION_PICKUP_POSE_BLUE = new Pose2d(0.6, 7.85, Rotation2d.fromDegrees(135));

    public static final Pose2d LEFT_STATION_PICKUP_POSE_RED = new Pose2d(
            FIELD_LENGTH - LEFT_STATION_PICKUP_POSE_BLUE.getX(),
            FIELD_HEIGHT - LEFT_STATION_PICKUP_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(-55));
    public static final Pose2d RIGHT_STATION_PICKUP_POSE_BLUE = new Pose2d(
            LEFT_STATION_PICKUP_POSE_BLUE.getX(),
            FIELD_HEIGHT - LEFT_STATION_PICKUP_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(-135));
    public static final Pose2d RIGHT_STATION_PICKUP_POSE_RED = new Pose2d(
            FIELD_LENGTH - RIGHT_STATION_PICKUP_POSE_BLUE.getX(),
            FIELD_HEIGHT - RIGHT_STATION_PICKUP_POSE_BLUE.getY(),
            Rotation2d.fromDegrees(55));

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static Translation2d getMark1() {
        return isBlueAlliance() ? BLUE_MARK_1 : RED_MARK_1;
    }

    public static Translation2d getMark2() {
        return isBlueAlliance() ? BLUE_MARK_2 : RED_MARK_2;
    }

    public static Translation2d getMark3() {
        return isBlueAlliance() ? BLUE_MARK_3 : RED_MARK_3;
    }

    public static Pose2d getFarLeftStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? FAR_LEFT_STARTING_POSE_BLUE : FAR_RIGHT_STARTING_POSE_RED;
    }

    public static Pose2d getFarRightStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? FAR_RIGHT_STARTING_POSE_BLUE : FAR_RIGHT_STARTING_POSE_RED;
    }

    public static Pose2d getLeftStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_STARTING_POSE_BLUE : LEFT_STARTING_POSE_RED;
    }

    public static Pose2d getRightStartingPose(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_STARTING_POSE_BLUE : RIGHT_STARTING_POSE_RED;
    }

    public static Pose2d getLeftStationPickup(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? LEFT_STATION_PICKUP_POSE_BLUE : LEFT_STATION_PICKUP_POSE_RED;
    }

    public static Pose2d getRightStationPickup(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? RIGHT_STATION_PICKUP_POSE_BLUE : RIGHT_STATION_PICKUP_POSE_RED;
    }

    public static Pose2d getFarLeftStartingPose() {
        return isBlueAlliance() ? FAR_LEFT_STARTING_POSE_BLUE : FAR_LEFT_STARTING_POSE_RED;
    }

    public static Pose2d getFarRightStartingPose() {
        return isBlueAlliance() ? FAR_RIGHT_STARTING_POSE_BLUE : FAR_RIGHT_STARTING_POSE_RED;
    }

    public static Pose2d getLeftStartingPose() {
        return isBlueAlliance() ? LEFT_STARTING_POSE_BLUE : LEFT_STARTING_POSE_RED;
    }

    public static Pose2d getRightStartingPose() {
        return isBlueAlliance() ? RIGHT_STARTING_POSE_BLUE : RIGHT_STARTING_POSE_RED;
    }

    public static Pose2d getLeftStationPickup() {
        return isBlueAlliance() ? LEFT_STATION_PICKUP_POSE_BLUE : LEFT_STATION_PICKUP_POSE_RED;
    }

    public static Pose2d getRightStationPickup() {
        return isBlueAlliance() ? RIGHT_STATION_PICKUP_POSE_BLUE : RIGHT_STATION_PICKUP_POSE_RED;
    }

    public static Pose3d getTagPose(int id) {
        if (id < RED_LEFT_CORAL_STATION || id > BLUE_LEFT_BARGE) {
            throw new IllegalArgumentException("id must be between 1 & 22");
        }

        return FIELD_LAYOUT.getTagPose(id).orElseThrow(() -> {
            final String msg = String.format("getTagPose called for unexpected tag %d", id);
            return new RuntimeException(msg);
        });
    }

    public static Pose2d getDesiredFinalScoringPoseForCoral(
            int tagID,
            Constants.SuperstructureConstants.ScoringSide scoringSide,
            Constants.SuperstructureConstants.ScoringDirection scoringDirection) {
        return getDesiredPointToDriveToForCoralScoring(tagID, scoringSide, scoringDirection, 0.0);
    }

    public static Pose2d getDesiredPointToDriveToForCoralScoring(
            int tagID,
            Constants.SuperstructureConstants.ScoringSide scoringSide,
            Constants.SuperstructureConstants.ScoringDirection scoringDirection,
            double distanceFromFinalScoringPose) {

        if (tagID >= 1 && tagID <= 22) {
            Pose2d tagPose = FieldConstants.FIELD_LAYOUT.getTagPose(tagID).get().toPose2d();
            double xOffset = Units.inchesToMeters(Constants.SuperstructureConstants.X_OFFSET_FROM_TAG_FOR_SCORING_INCHES
                    + Units.metersToInches(distanceFromFinalScoringPose));

            double yOffset = -Units.inchesToMeters(
                    Constants.SuperstructureConstants.Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES);
            if (scoringSide == Constants.SuperstructureConstants.ScoringSide.RIGHT) {
                yOffset *= -1;
            }
            Translation2d offsetFromTag = new Translation2d(xOffset, yOffset);

            var transformedPose =
                    tagPose.plus(new Transform2d(offsetFromTag.getX(), offsetFromTag.getY(), Rotation2d.kZero));

            if (scoringDirection == Constants.SuperstructureConstants.ScoringDirection.FRONT) {
                transformedPose = new Pose2d(
                        transformedPose.getTranslation(),
                        transformedPose.getRotation().plus(Rotation2d.k180deg));
            }

            return transformedPose;
        } else {
            return Pose2d.kZero;
        }
    }
}
