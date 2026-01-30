package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

        angleConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .inverted(moduleConfig.invertInternalAngle())
                .voltageCompensation(12.0);

        angleConfig.absoluteEncoder
                .positionConversionFactor(DriveConstants.ANGLE_POSITION_CONVERSION_FACTOR)
                .zeroOffset(Units.degreesToRotations(moduleConfig.absoluteEncoderOffset()))
                .inverted(moduleConfig.invertAngle())
                .averageDepth(2);

        angleConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(DriveConstants.ANGLE_P.get(), 0.0, DriveConstants.ANGLE_D.get())
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, 2 * Math.PI);  // Wrapping range in radians

        angleConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        driveConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40)
                .voltageCompensation(12.0);

        driveConfig.encoder
                .positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(DriveConstants.DRIVE_P.get(), 0.0, DriveConstants.DRIVE_D.get())
                .outputRange(-1, 1);

        driveConfig.signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        driveConfig.inverted(moduleConfig.invertDrive());

        this.driveMotor = new SparkFlex(moduleConfig.driveID(), SparkLowLevel.MotorType.kBrushless);
        this.angleMotor = new SparkMax(moduleConfig.angleID(), SparkLowLevel.MotorType.kBrushless);

        tryConfigureWithRetry(driveMotor, driveConfig, "Drive motor " + moduleNumber);
        tryConfigureWithRetry(angleMotor, angleConfig, "Angle motor " + moduleNumber);

        angleMotor.getEncoder().setPosition(angleMotor.getAbsoluteEncoder().getPosition());

        driveMotor.getEncoder().setPosition(0.0);

        this.driveController = driveMotor.getClosedLoopController();
        this.angleController = angleMotor.getClosedLoopController();

        var odometryThread = SparkOdometryThread.getInstance();
        this.drivePositionQueue = odometryThread.registerSignal(driveMotor, () -> driveMotor.getEncoder().getPosition());
        this.anglePositionQueue = odometryThread.registerSignal(angleMotor, () -> angleMotor.getAbsoluteEncoder().getPosition());
        this.timestampQueue = odometryThread.makeTimestampQueue();

        this.moduleOffset = moduleConfig.moduleOffset();
        this.moduleNumber = moduleConfig.moduleNumber();
    }

    /**
     * Retry configuration up to 5 times to handle CAN bus flakiness
     */
    private void tryConfigureWithRetry(SparkBase spark, Object config, String name) {
        for (int i = 0; i < 5; i++) {
            REVLibError error;
            if (config instanceof SparkFlexConfig) {
                error = ((SparkFlex) spark).configure(
                        (SparkFlexConfig) config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters
                );
            } else {
                error = ((SparkMax) spark).configure(
                        (SparkMaxConfig) config,
                        ResetMode.kResetSafeParameters,
                        PersistMode.kPersistParameters
                );
            }

            if (error == REVLibError.kOk) {
                Logger.recordOutput("SwerveModule/" + name + "/ConfigSuccess", true);
                return;
            }

            Logger.recordOutput("SwerveModule/" + name + "/ConfigRetry", i + 1);
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        Logger.recordOutput("SwerveModule/" + name + "/ConfigFailed", true);
        new Alert(name + " configuration failed after 5 retries!", AlertType.kError).set(true);
    }

    public void periodic() {
        String modulePrefix = "Module" + moduleNumber + "/";

        // ============ LOGGING ============
        double currentAngleRadians = angleMotor.getAbsoluteEncoder().getPosition();
        double angleSetpoint = angleController.getSetpoint();
        Logger.recordOutput(modulePrefix + "AngleCurrent_rad", currentAngleRadians);
        Logger.recordOutput(modulePrefix + "AngleSetpoint_rad", angleSetpoint);
        Logger.recordOutput(modulePrefix + "AngleError_rad", angleSetpoint - currentAngleRadians);
        Logger.recordOutput(modulePrefix + "AngleCurrent_deg", Units.radiansToDegrees(currentAngleRadians));
        Logger.recordOutput(modulePrefix + "AngleInternalCurrent_deg", angleMotor.getEncoder().getPosition());
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

                    driveUpdateConfig.encoder
                            .positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR)
                            .velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR)
                            .uvwMeasurementPeriod(10)
                            .uvwAverageDepth(2);

                    driveUpdateConfig.closedLoop
                            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                            .pid(DriveConstants.DRIVE_P.get(), 0.0, DriveConstants.DRIVE_D.get())
                            .outputRange(-1, 1);

                    driveUpdateConfig.inverted(moduleConfig.invertDrive());

                    driveMotor.configure(
                            driveUpdateConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters
                    );

                    driveFeedforward = new SimpleMotorFeedforward(
                            DriveConstants.DRIVE_KS.get(),
                            DriveConstants.DRIVE_KV.get(),
                            DriveConstants.DRIVE_KA.get()
                    );

                    Logger.recordOutput("Drive PID updated for module", moduleNumber);
                },
                DriveConstants.DRIVE_P, DriveConstants.DRIVE_D, DriveConstants.DRIVE_KS, DriveConstants.DRIVE_KV, DriveConstants.DRIVE_KA
        );

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    SparkMaxConfig angleUpdateConfig = new SparkMaxConfig();

                    angleUpdateConfig.absoluteEncoder
                            .positionConversionFactor(DriveConstants.ANGLE_POSITION_CONVERSION_FACTOR)
                            .zeroOffset(Units.degreesToRotations(DriveConstants.ANGLE_OFFSETS[moduleNumber].get()))
                            .averageDepth(2);

                    angleUpdateConfig.closedLoop
                            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                            .pid(DriveConstants.ANGLE_P.get(), 0.0, DriveConstants.ANGLE_D.get())
                            .outputRange(-1, 1)
                            .positionWrappingEnabled(true)
                            .positionWrappingInputRange(0, 2 * Math.PI);

                    angleUpdateConfig.inverted(moduleConfig.invertAngle());

                    angleMotor.configure(
                            angleUpdateConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters
                    );

                    Logger.recordOutput("Angle PID updated for module", moduleNumber);
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
                new Rotation2d(angleMotor.getAbsoluteEncoder().getPosition())
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
        String modulePrefix = "Module" + moduleNumber + "/";

        // Log BEFORE optimization
        Logger.recordOutput(modulePrefix + "DesiredAngle_deg", desiredState.angle.getDegrees());
        Logger.recordOutput(modulePrefix + "DesiredSpeed_mps", desiredState.speedMetersPerSecond);
        Logger.recordOutput(modulePrefix + "CurrentAngle_deg", getState().angle.getDegrees());

        // Optimize in place (new way)
        // desiredState.optimize(getState().angle);

        // Log AFTER optimization
        Logger.recordOutput(modulePrefix + "OptimizedAngle_deg", desiredState.angle.getDegrees());
        Logger.recordOutput(modulePrefix + "OptimizedSpeed_mps", desiredState.speedMetersPerSecond);

        // Apply cosine scaling
        desiredState.cosineScale(getState().angle);

        driveVelocityVoltage(desiredState.speedMetersPerSecond);
        setAnglePosition(desiredState.angle);
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

    private void setAnglePosition(Rotation2d angle) {
        double setpoint = MathUtil.inputModulus(angle.getRadians(), 0, 2 * Math.PI);

        String modulePrefix = "Module" + moduleNumber + "/";
        Logger.recordOutput(modulePrefix + "TargetAngle_rad", angle.getRadians());
        Logger.recordOutput(modulePrefix + "TargetAngle_deg", angle.getDegrees());
        Logger.recordOutput(modulePrefix + "AngleSetpointSent_rad", setpoint);

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