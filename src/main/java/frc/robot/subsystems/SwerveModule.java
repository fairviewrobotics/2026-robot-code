package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
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

public class SwerveModule extends SubsystemBase {

    int moduleNumber;
    SparkFlex driveMotor;
    SparkMax angleMotor;
    Translation2d moduleOffset;
    private final SparkClosedLoopController driveController;
    private final SparkClosedLoopController angleController;
    SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_KS.get(), DriveConstants.DRIVE_KV.get(), DriveConstants.DRIVE_KA.get());
    SimpleMotorFeedforward angleFeedforward;
    public double[] odometryTimestamps = new double[] {};
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

    public SwerveModule(SwerveModuleConfig moduleConfig) {

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        SparkFlexConfig driveConfig = new SparkFlexConfig();

        angleConfig.absoluteEncoder.positionConversionFactor(DriveConstants.ANGLE_GEAR_RATIO);
        angleConfig.closedLoop.pid(DriveConstants.ANGLE_P.get(), 0.0, DriveConstants.ANGLE_D.get());
        angleConfig.inverted(moduleConfig.invertAngle());
        angleConfig.absoluteEncoder.zeroOffset(moduleConfig.absoluteEncoderOffset());

        driveConfig.encoder.positionConversionFactor(DriveConstants.DRIVE_POSITION_CONVERSION_FACTOR);
        driveConfig.encoder.velocityConversionFactor(DriveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        driveConfig.closedLoop.pid(DriveConstants.DRIVE_P.get(), 0.0, DriveConstants.DRIVE_D.get());
        driveConfig.inverted(moduleConfig.invertDrive());

        this.driveController = driveMotor.getClosedLoopController();
        this.angleController = angleMotor.getClosedLoopController();

        this.driveMotor = new SparkFlex(moduleConfig.driveID(), SparkLowLevel.MotorType.kBrushless);
        this.angleMotor = new SparkMax(moduleConfig.angleID(), SparkLowLevel.MotorType.kBrushless);

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        angleMotor.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        this.moduleNumber = moduleConfig.moduleNumber();

    }

    public void periodic() {

            TunableNumber.ifChanged(
                    hashCode(),
                    () -> {
                        SparkFlexConfig driveUpdateConfig = new SparkFlexConfig();

                        driveUpdateConfig.closedLoop.pid(
                                DriveConstants.DRIVE_P.get(),
                                0.0,
                                DriveConstants.DRIVE_D.get()
                        );

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
                    SparkFlexConfig angleUpdateConfig = new SparkFlexConfig();

                    angleUpdateConfig.closedLoop.pid(
                            DriveConstants.DRIVE_P.get(),
                            0.0,
                            DriveConstants.DRIVE_D.get()
                    );
                    angleUpdateConfig.absoluteEncoder.zeroOffset(DriveConstants.ANGLE_OFFSETS[moduleNumber].get());

                    angleMotor.configure(
                            angleUpdateConfig,
                            ResetMode.kNoResetSafeParameters,
                            PersistMode.kNoPersistParameters
                    );

                    Logger.recordOutput("Angle PID updated for", moduleNumber);
                },
                DriveConstants.ANGLE_P, DriveConstants.ANGLE_D, DriveConstants.ANGLE_OFFSETS[moduleNumber]
        );

        // Calculate positions for odometry
        int sampleCount = odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = odometryDrivePositionsRad[i] * Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER_INCHES);
            Rotation2d angle = odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
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
                driveMotor.getEncoder().getVelocity() * (DriveConstants.WHEEL_DIAMETER_INCHES/2),
                new Rotation2d(angleMotor.getAbsoluteEncoder().getPosition()));
    }

    public Pose2d getModulePose() {
        return new Pose2d(moduleOffset, new Rotation2d(angleMotor.getAbsoluteEncoder().getPosition()));
    }

    public double[] getOdometryTimestamps() {
        return odometryTimestamps;
    }

    public void teleopInit() {
        setAnglePositionVoltage(0.0);
    }

    public void resetDriveEncoder() {
        driveMotor.getEncoder().setPosition(0.0);
    }

    public void setVelocity(SwerveModuleState desiredState) {
        driveVelocityVoltage(desiredState.speedMetersPerSecond);
        setAnglePositionVoltage(desiredState.angle.getRadians());
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

    private void setAnglePositionVoltage(double radians) {
        angleController.setSetpoint(
                radians,
                SparkBase.ControlType.kPosition,
                ClosedLoopSlot.kSlot0
        );
    }

    public void stop() {
        angleMotor.set(0.0);
        driveMotor.set(0.0);
    }

}
