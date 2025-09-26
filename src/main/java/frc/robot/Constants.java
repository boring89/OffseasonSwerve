// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.Swerve.ModuleLimits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants {

    public static final double kMaxSpeedMeterPerSecond = 4;
    public static final double kMaxAngularSpeedRadiansPerSecond = 3 * 1.8 * Math.PI;

    public static final double kMaxAccerationUnitsPerSecond = 7;

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
}
