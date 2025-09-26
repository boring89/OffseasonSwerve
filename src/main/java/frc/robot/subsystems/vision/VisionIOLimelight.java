package frc.robot.subsystems.vision;

import java.lang.module.Configuration;
import java.util.ArrayList;
import java.util.concurrent.TransferQueue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.configs.CameraConfiguration;

import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
    
    private static final double TAG_HEIGHT = Units.inchesToMeters(6.5);
    private static final double HEIGHT_OF_TAG_OFF_GROUND = Units.inchesToMeters(8.87);
    
    public static final double VERTICAL_RESOLUTION_IN_PIXELS = 800;
    public static final double VERTICAL_FIELD_OF_VIEW_IN_DEGREES = 56.2;
    public static final double LIMELIGHT_LATENCY_CONSTANT_SECONDS = 0.0;

    public static final double LIMELIGHT_SCALAR_FACTOR = 38.12 / 35.99; //TODO


    private final NetworkTableEntry valid, tx, tid, corner, throttleSet;

    private final ArrayList<Pair<Double, Double>> cornerListPairs;

    private final String limelightName;
    private final CameraConfiguration.Location limelightLocation;
    private final Translation2d cameraToRobotCenter;
    private final double offsetBetweenApriltagBottomAndCamera;
    private final Rotation2d limelightYaw;
    private final double distanceScalarValue;

    public VisionIOLimelight(CameraConfiguration conf) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        this.limelightYaw = Rotation2d.fromRadians(conf.LimelightMountingYawRad);
        this.limelightName = conf.getName();
        this.limelightLocation = conf.location;
        this.cameraToRobotCenter = conf.getTranslationToRobotCenter();
        this.offsetBetweenApriltagBottomAndCamera = HEIGHT_OF_TAG_OFF_GROUND - conf.LimelightHeightOffsetMeters;
        this.distanceScalarValue = conf.LimelightDistanceScalarValue;

        this.valid = inst.getTable(limelightName).getEntry("tv");
        this.tx = inst.getTable(limelightName).getEntry("tx");
        this.tid = inst.getTable(limelightName).getEntry("tid");
        this.corner = inst.getTable(limelightName).getEntry("tcornxy");
        this.throttleSet = inst.getTable(limelightName).getEntry("throttle_set");

        this.cornerListPairs = new ArrayList<Pair<Double, Double>>();
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.horizontalAngleToTarget = Rotation2d.fromDegrees(tx.getDouble(HEIGHT_OF_TAG_OFF_GROUND));
        inputs.hasTarget = valid.getDouble(0.0) == 1.0;
        inputs.tagId = (int) tid.getDouble(-1);
        
        this.cornerListPairs.clear();

        var x = corner.getDoubleArray(new double[0]);

        if (x.length >= 8) {
            for (int i = 0; i < 7; i += 2) {
                this.cornerListPairs.add(new Pair<Double,Double>(x[i], x[i+1]));
            }
        }

        Logger.recordOutput(this.limelightName + "/Number of corners", this.cornerListPairs.size());

        if (inputs.hasTarget && cornerListPairs.size() >= 4) {
            inputs.targetHeight = calculateTargetHeightInPixels(this.cornerListPairs);
            inputs.angleEncompassingTag = calculateAngleEncompassingTagHeight(inputs.targetHeight);
            inputs.distanceToTagMeters = calculateDistanceToAprilTagInMetersUsingTrigMethod(inputs)
        }
    }
}
