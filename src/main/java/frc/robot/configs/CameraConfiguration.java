package frc.robot.configs;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraConfiguration {
    
    public enum Location {
        FL,
        FR,
        NONE
    }

    private final String name;
    public double LimelightMountingRollRad = 0;
    public double LimelightMountingYawRad = 0;
    public double LimelightMountingPitchRad = 0;
    public double LimelightHeightOffsetMeters = 0;
    public double LimelightLengthOffsetMeters = 0;
    public double LimelightWidthOffsetMeters = 0;
    public Location location = Location.NONE;

    public double LimelightDistanceScalarValue = 0;

    public CameraConfiguration(String name) {
        this.name = name;
    }

    public CameraConfiguration() {
        this("limelight");
    }

    public String getName() {
        return name;
    }

    public CameraConfiguration withMountingRoll(double rollRad) {
        this.LimelightMountingRollRad = rollRad;
        return this;
    }

    public CameraConfiguration withMountingYaw(double yawRad) {
        this.LimelightMountingYawRad = yawRad;
        return this;
    }

    public CameraConfiguration withMountingPitch(double pitchRad) {
        this.LimelightMountingPitchRad = pitchRad;
        return this;
    }

    public CameraConfiguration withHeightOffset(double heightOffsetMeters) {
        this.LimelightHeightOffsetMeters = heightOffsetMeters;
        return this;
    }

    public CameraConfiguration withLengthOffset(double lengthOffsetMeters) {
        this.LimelightLengthOffsetMeters = lengthOffsetMeters;
        return this;
    }

    public CameraConfiguration withWidthOffset(double widthOffsetMeters) {
        this.LimelightWidthOffsetMeters = widthOffsetMeters;
        return this;
    }

    public CameraConfiguration withDistanceScalar(double distanceScalar) {
        this.LimelightDistanceScalarValue = distanceScalar;
        return this;
    }

    public Translation3d getTranslationOffset() {
        return new Translation3d(
            this.LimelightLengthOffsetMeters,
            this.LimelightHeightOffsetMeters, 
            this.LimelightWidthOffsetMeters
        );
    }

    public Rotation3d getRotationOffset() {
        return new Rotation3d(
            this.LimelightMountingRollRad,
            this.LimelightMountingPitchRad,
            this.LimelightMountingYawRad
        );
    }

    public Translation2d getTranslationToRobotCenter() {
        return new Translation2d(this.LimelightLengthOffsetMeters, this.LimelightWidthOffsetMeters);
    }

    public Location getLocation() {
        return this.location;
    }
}
