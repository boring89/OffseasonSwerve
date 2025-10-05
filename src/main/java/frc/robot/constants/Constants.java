// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.Swerve.ModuleLimits;

public final class Constants {

  public static class DriveConstants {

    public static final double kMaxSpeedMeterPerSecond = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3 * 1.8 * Math.PI;

    public static final double kMaxAccerationUnitsPerSecond = 5;

    public static final int[] kDriveMotorID = { 1, 2, 3, 4 };
    public static final int[] kTurnMotorID = { 5, 6, 7, 8 };
    public static final int[] kCANcoderID = { 9, 10, 11, 12 };

    public static final Translation2d[] moduleLocations = new Translation2d[] {
        new Translation2d(0.278, 0.278),
        new Translation2d(0.278, -0.278),
        new Translation2d(-0.278, 0.278),
        new Translation2d(-0.278, -0.278)
    };

    public static final ModuleLimits moduleLimitsFree = new ModuleLimits(kMaxSpeedMeterPerSecond,
        kMaxAccerationUnitsPerSecond, Units.degreesToRadians(1080.0));
  }

  public static final class ModuleConstants {
    public static final double WilliamConstant = 1.042;
    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.10068;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
    public static final double kTurningGearRaitio = 1 / 19.6;
  }

  public static final class MotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class OIConstants {
    public static final double kDeadband = 0.05;
  }

  public static final class TagConstants {
    public static final AprilTagFieldLayout FIELD_LAYOUT =  AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }







  // from 2910's code
  public static final class SuperstructureConstants {
    public static final double X_OFFSET_FROM_TAG_FOR_L1_BASE_SCORING_INCHES = 21.0;
    public static final double X_OFFSET_FROM_TAG_FOR_L1_TOP_SCORING_INCHES = 19.25;

    public static final double X_OFFSET_FROM_TAG_FOR_SCORING_INCHES = 22.0;
    public static final double X_OFFSET_FROM_TAG_FOR_INTAKING_ALGAE_INCHES = 18.0;
    public static final double X_OFFSET_FROM_TAG_FOR_INTERMEDIATE_INTAKING_ALGAE_INCHES = 30.0;
    public static final double X_OFFSET_FROM_TAG_FOR_BACKOUT_INTAKING_ALGAE_INCHES = 50.0;
    public static final double X_OFFSET_FROM_TAG_FOR_L1_BACKOUT_INCHES = 10.0;

    public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES = 6.5;
    public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_L1_INCHES = 9.0;

    public enum ScoringDirection {
      FRONT,
      BACK
    }

    public enum ScoringSide {
      RIGHT,
      LEFT
    }

    public enum AutomationLevel {
      AUTO_RELEASE,
      AUTO_DRIVE_AND_MANUAL_RELEASE,
      NO_AUTO_DRIVE
    }

    public enum ReefSelectionMethod {
      POSE,
      ROTATION
    }
  }

  public static final class ReefConstants {
    public enum ReefFaces {
      AB,
      CD,
      EF,
      GH,
      IJ,
      KL
    }

    public enum AlgaeIntakeLocation {
      L2,
      L3
    }

    public static final class AlgaeIntakeMapping {
      public final AlgaeIntakeLocation FRONT;
      public final AlgaeIntakeLocation BACK;

      public AlgaeIntakeMapping(AlgaeIntakeLocation front, AlgaeIntakeLocation back) {
        FRONT = front;
        BACK = back;
      }
    }

    public static final class ScoringCoralMappingRotationToTagID {
      public final int FRONT_ID;
      public final int BACK_ID;

      public ScoringCoralMappingRotationToTagID(int frontID, int backID) {
        FRONT_ID = frontID;
        BACK_ID = backID;
      }
    }

    public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap = Map.of(
        FieldConstants.getTagPose(21).toPose2d(), 21,
        FieldConstants.getTagPose(20).toPose2d(), 20,
        FieldConstants.getTagPose(19).toPose2d(), 19,
        FieldConstants.getTagPose(18).toPose2d(), 18,
        FieldConstants.getTagPose(17).toPose2d(), 17,
        FieldConstants.getTagPose(22).toPose2d(), 22);

    public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap = Map.of(
        FieldConstants.getTagPose(6).toPose2d(), 6,
        FieldConstants.getTagPose(7).toPose2d(), 7,
        FieldConstants.getTagPose(8).toPose2d(), 8,
        FieldConstants.getTagPose(9).toPose2d(), 9,
        FieldConstants.getTagPose(10).toPose2d(), 10,
        FieldConstants.getTagPose(11).toPose2d(), 11);

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> redAllianceAngleToTagIDsMap = Map.of(
        Rotation2d.fromDegrees(-60),
        new ScoringCoralMappingRotationToTagID(9, 6),
        Rotation2d.fromDegrees(-120),
        new ScoringCoralMappingRotationToTagID(8, 11),
        Rotation2d.k180deg,
        new ScoringCoralMappingRotationToTagID(7, 10),
        Rotation2d.fromDegrees(120),
        new ScoringCoralMappingRotationToTagID(6, 9),
        Rotation2d.fromDegrees(60),
        new ScoringCoralMappingRotationToTagID(11, 8),
        Rotation2d.kZero,
        new ScoringCoralMappingRotationToTagID(10, 7));

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> blueAllianceAngleToTagIDsMap = Map.of(
        Rotation2d.fromDegrees(-60),
        new ScoringCoralMappingRotationToTagID(19, 22),
        Rotation2d.fromDegrees(-120),
        new ScoringCoralMappingRotationToTagID(20, 17),
        Rotation2d.k180deg,
        new ScoringCoralMappingRotationToTagID(21, 18),
        Rotation2d.fromDegrees(120),
        new ScoringCoralMappingRotationToTagID(22, 19),
        Rotation2d.fromDegrees(60),
        new ScoringCoralMappingRotationToTagID(17, 20),
        Rotation2d.kZero,
        new ScoringCoralMappingRotationToTagID(18, 21));

    public static final Map<Rotation2d, AlgaeIntakeMapping> redAllianceAlgae = Map.of(
        Rotation2d.fromDegrees(0),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(60),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(120),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(180),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(-120),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(-60),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2));

    public static final Map<Rotation2d, AlgaeIntakeMapping> blueAllianceAlgae = Map.of(
        Rotation2d.fromDegrees(0),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(60),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(120),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(180),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(-120),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(-60),
        new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3));

    public static final Map<ReefFaces, Integer> redAllianceReefFacesToIds = Map.of(
        ReefFaces.AB, 7,
        ReefFaces.CD, 8,
        ReefFaces.EF, 9,
        ReefFaces.GH, 10,
        ReefFaces.IJ, 11,
        ReefFaces.KL, 6);

    public static final Map<ReefFaces, Integer> blueAllianceReefFacesToIds = Map.of(
        ReefFaces.AB, 18,
        ReefFaces.CD, 17,
        ReefFaces.EF, 22,
        ReefFaces.GH, 21,
        ReefFaces.IJ, 20,
        ReefFaces.KL, 19);
  }
}
