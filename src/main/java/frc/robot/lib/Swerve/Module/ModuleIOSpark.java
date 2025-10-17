package frc.robot.lib.Swerve.Module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.fasterxml.jackson.databind.cfg.CoercionInputShape;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.constants.Constants.DriveConstants;

public class ModuleIOSpark extends ModuleIO {

    private final SparkFlex driveMotor;
    private final SparkMax turnMotor;

    private final SparkClosedLoopController drivePIDController, turnPIDController;

    private final CANcoder absoluteEncoder;

    private final ModuleIOInputs inputs = new ModuleIOInputs();

    public ModuleIOSpark(int ModuleId, boolean ModuleReverse) {
        this.driveMotor = new SparkFlex(DriveConstants.kDriveMotorID[ModuleId], MotorType.kBrushless);
        this.turnMotor = new SparkMax(DriveConstants.kTurnMotorID[ModuleId], MotorType.kBrushless);

        this.driveMotor.configure(Configs.drivingConfig.inverted(ModuleReverse), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.turnMotor.configure(Configs.turningConfig.inverted(true), ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        this.drivePIDController = driveMotor.getClosedLoopController();
        this.turnPIDController = turnMotor.getClosedLoopController();

        this.absoluteEncoder = new CANcoder(DriveConstants.kCANcoderID[ModuleId]);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePosition = driveMotor.getEncoder().getPosition();
        inputs.turnPosition = turnMotor.getEncoder().getPosition();

        inputs.driveVelocity = driveMotor.getEncoder().getVelocity();
        inputs.turnVelocity = turnMotor.getEncoder().getVelocity();

        inputs.absoluteAngle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;

        inputs.modulePosition = new SwerveModulePosition(inputs.drivePosition,
                Rotation2d.fromRadians(inputs.turnPosition));
        inputs.moduleState = new SwerveModuleState(inputs.driveVelocity, Rotation2d.fromRadians(inputs.absoluteAngle));
    }

    @Override
    public void setModuleState(SwerveModuleState state) {
        state = ModuleIO.optimizeState(state, inputs.turnPosition);
        drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
        turnPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    @Override
    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        turnMotor.getEncoder().setPosition(absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
    }

}
