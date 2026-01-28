package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class SwerveModule extends SubsystemBase {

    int moduleNumber;
    SparkFlex driveMotor;
    SparkMax angleMotor;
    Translation2d moduleOffset;
    private final SwerveModuleConfig moduleConfig;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS.get(), DriveConstants.DRIVE_KV.get(), DriveConstants.DRIVE_KA.get());
    SimpleMotorFeedforward angleFeedforward;
    public double[] odometryTimestamps = new double[] {};
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> anglePositionQueue;
    private final Queue<Double> timestampQueue;

    public SwerveModule(SwerveModuleConfig moduleConfig) {
        this.moduleConfig = moduleConfig;

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkFlexConfig driveConfig = new SparkFlexConfig();

        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        angleConfig.absoluteEncoder.positionConversionFactor(DriveConstants.ANGLE_POSITION_CONVERSION_FACTOR);
        angleConfig.closedLoop.positionWrappingEnabled(true);
        angleConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
        angleConfig.closedLoop.pid(DriveConstants.ANGLE_P.get(), 0.0, DriveConstants.ANGLE_D.get());
        angleConfig.inverted(moduleConfig.invertAngle());
        angleConfig.absoluteEncoder.zeroOffset(Units.degreesToRotations(moduleConfig.absoluteEncoderOffset()));

        driveConfig.encoder.positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveConfig.closedLoop.pid(DriveConstants.DRIVE_P.get(), 0.0, DriveConstants.DRIVE_D.get());
        driveConfig.inverted(moduleConfig.invertDrive());

        this.driveMotor = new SparkFlex(moduleConfig.driveID(), SparkLowLevel.MotorType.kBrushless);
        this.angleMotor = new SparkMax(moduleConfig.angleID(), SparkLowLevel.MotorType.kBrushless);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Sync relative encoder to absolute encoder AFTER configuration
        angleMotor.getEncoder().setPosition(angleMotor.getAbsoluteEncoder().getPosition());

        this.driveController = driveMotor.getClosedLoopController();
        this.angleController = angleMotor.getClosedLoopController();

        var odometryThread = SparkOdometryThread.getInstance();
        this.drivePositionQueue = odometryThread.registerSignal(driveMotor, () -> driveMotor.getEncoder().getPosition());
        this.anglePositionQueue = odometryThread.registerSignal(angleMotor, () -> angleMotor.getAbsoluteEncoder().getPosition());
        this.timestampQueue = odometryThread.makeTimestampQueue();

        this.moduleOffset = moduleConfig.moduleOffset();
        this.moduleNumber = moduleConfig.moduleNumber();

    }

    public void periodic() {
        String modulePrefix = "Module" + moduleNumber + "/";

        double currentAngleRadians = angleMotor.getAbsoluteEncoder().getPosition();
        double angleSetpoint = angleController.getSetpoint();
        Logger.recordOutput(modulePrefix + "AngleCurrent_rot", currentAngleRadians);
        Logger.recordOutput(modulePrefix + "AngleSetpoint_rot", angleSetpoint);
        Logger.recordOutput(modulePrefix + "AngleError_rot", angleSetpoint - currentAngleRadians);
        Logger.recordOutput(modulePrefix + "AngleCurrent_deg", Units.radiansToDegrees(currentAngleRadians));
        Logger.recordOutput(modulePrefix + "AngleSetpoint_deg", Units.radiansToDegrees(angleSetpoint));

        double currentVelocity = driveMotor.getEncoder().getVelocity();
        double velocitySetpoint = driveController.getSetpoint();
        Logger.recordOutput(modulePrefix + "DriveVelocityCurrent_mps", currentVelocity);
        Logger.recordOutput(modulePrefix + "DriveVelocitySetpoint_mps", velocitySetpoint);
        Logger.recordOutput(modulePrefix + "DriveVelocityError_mps", velocitySetpoint - currentVelocity);
        Logger.recordOutput(modulePrefix + "DrivePosition_m", driveMotor.getEncoder().getPosition());

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    SparkFlexConfig driveUpdateConfig = new SparkFlexConfig();
                    driveUpdateConfig.encoder.positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
                    driveUpdateConfig.encoder.velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
                    driveUpdateConfig.closedLoop.pid(
                            DriveConstants.DRIVE_P.get(),
                            0.0,
                            DriveConstants.DRIVE_D.get()
                    );
                    driveUpdateConfig.inverted(moduleConfig.invertDrive());

                    driveMotor.configure(
                            driveUpdateConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters
                    );

                    driveFeedforward.setKs(DriveConstants.DRIVE_KS.get());
                    driveFeedforward.setKv(DriveConstants.DRIVE_KV.get());
                    driveFeedforward.setKa(DriveConstants.DRIVE_KA.get());

                    Logger.recordOutput("Drive PID updated for", moduleNumber);
                },
                DriveConstants.DRIVE_P, DriveConstants.DRIVE_D, DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA
        );

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    SparkMaxConfig angleUpdateConfig = new SparkMaxConfig();

                    angleUpdateConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                    angleUpdateConfig.absoluteEncoder.positionConversionFactor(DriveConstants.ANGLE_POSITION_CONVERSION_FACTOR);
                    angleUpdateConfig.closedLoop.positionWrappingEnabled(true);
                    angleUpdateConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);
                    angleUpdateConfig.inverted(moduleConfig.invertAngle());
                    angleUpdateConfig.closedLoop.pid(
                            DriveConstants.ANGLE_P.get(),
                            0.0,
                            DriveConstants.ANGLE_D.get()
                    );
                    angleUpdateConfig.absoluteEncoder.zeroOffset(Units.degreesToRotations(DriveConstants.ANGLE_OFFSETS[moduleNumber].get()));

                    angleMotor.configure(
                            angleUpdateConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters
                    );

                    Logger.recordOutput("Angle PID updated for", moduleNumber);
                },
                DriveConstants.ANGLE_P, DriveConstants.ANGLE_D, DriveConstants.ANGLE_OFFSETS[moduleNumber]
        );

        int sampleCount = timestampQueue.size();
        odometryTimestamps = new double[sampleCount];
        odometryDrivePositionsRad = new double[sampleCount];
        odometryTurnPositions = new Rotation2d[sampleCount];

        for (int i = 0; i < sampleCount; i++) {
            odometryTimestamps[i] = timestampQueue.poll();
            odometryDrivePositionsRad[i] = drivePositionQueue.poll();
            odometryTurnPositions[i] = Rotation2d.fromRadians(anglePositionQueue.poll());
        }
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                driveMotor.getEncoder().getPosition(),
                new Rotation2d(this.getModulePose().getRotation().getRadians())
        );
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveMotor.getEncoder().getVelocity(),
                new Rotation2d(angleMotor.getAbsoluteEncoder().getPosition()));
    }

    public Pose2d getModulePose() {
        return new Pose2d(moduleOffset, new Rotation2d(angleMotor.getAbsoluteEncoder().getPosition()));
    }

    public double[] getOdometryTimestamps() {
        return odometryTimestamps;
    }

    public SwerveModulePosition getOdometryPosition(int index) {
        return new SwerveModulePosition(
                odometryDrivePositionsRad[index],
                odometryTurnPositions[index]
        );
    }

    public void resetDriveEncoder() {
        driveMotor.getEncoder().setPosition(0.0);
    }

    public void setVelocity(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(
                desiredState,
                getState().angle
        );

        driveVelocityVoltage(optimizedState.speedMetersPerSecond);
        setAnglePositionRadians(optimizedState.angle);
    }

    private void driveVelocityVoltage(double speedMetersPerSecond) {

        double ffVoltage = driveFeedforward.calculate(speedMetersPerSecond);

        driveController.setSetpoint(
                speedMetersPerSecond,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVoltage,
                SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    private void setAnglePositionRadians(Rotation2d angle) {

        double setpoint = MathUtil.inputModulus(angle.getRadians(), 0, 2 * Math.PI);

        String modulePrefix = "Module" + moduleNumber + "/";
        Logger.recordOutput(modulePrefix + "target-angle", angle);
        Logger.recordOutput(modulePrefix + "angle-error", angle.getRadians() - angleMotor.getAbsoluteEncoder().getPosition());
        Logger.recordOutput(modulePrefix + "AngleOptimizedSetpoint_rot", setpoint);

        angleController.setSetpoint(
                setpoint,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0
        );
    }

    public void stop() {
        angleMotor.set(0.0);
        driveMotor.set(0.0);
    }

}