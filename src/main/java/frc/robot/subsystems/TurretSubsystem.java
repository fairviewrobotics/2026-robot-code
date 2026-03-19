package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final ProfiledPIDController turretPID = new ProfiledPIDController(
            ShootingConstants.TURRET_P,
            0.0,
            ShootingConstants.TURRET_D,
            ShootingConstants.TURRET_CONSTRAINTS);

    private final SparkFlex turretMotor = new SparkFlex(ShootingConstants.TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlexConfig turretMotorConfig = new SparkFlexConfig();
    private final SoftLimitConfig turretSoftLimits = new SoftLimitConfig();
    private final DigitalInput turretSwitch = new DigitalInput(0);

    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
            ShootingConstants.TURRET_KS,
            ShootingConstants.TURRET_KV,
            ShootingConstants.TURRET_KA
    );

    private final InterpolatingDoubleTreeMap AngleToRPMScalarMap =
            new InterpolatingDoubleTreeMap();

    private boolean isZeroed = false;

    private double lastKP = ShootingConstants.TURRET_P;
    private double lastKD = ShootingConstants.TURRET_D;
    private double lastKS = ShootingConstants.TURRET_KS;
    private double lastKV = ShootingConstants.TURRET_KV;

    private boolean snappingBack = false;
    private boolean leftOfKicker = false;

    public TurretSubsystem() {

        initializePreferences();

//        LimitSwitchConfig turretLimitSwitchConfig = new LimitSwitchConfig();
//
//        turretLimitSwitchConfig
//                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotorAndSetPosition)
//                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        turretSoftLimits
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(Units.degreesToRadians(ShootingConstants.TURRET_FORWARD_LIMIT_DEGREES))
                .reverseSoftLimit(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES));

        turretMotorConfig
                .inverted(false)
                // .apply(turretLimitSwitchConfig)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .encoder.positionConversionFactor(ShootingConstants.TURRET_ENCODER_TO_RADIANS_CONVERSION_FACTOR);

        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        updateCache();
        turretPID.setTolerance(Units.degreesToRadians(0.05));
        createAngleRPMToScalarMap();
    }

    private void initializePreferences() {
        Preferences.initDouble("Turret/kP", ShootingConstants.TURRET_P);
        Preferences.initDouble("Turret/kD", ShootingConstants.TURRET_D);
        Preferences.initDouble("Turret/kV", ShootingConstants.TURRET_KS);
        Preferences.initDouble("Turret/kS", ShootingConstants.TURRET_KS);
    }

    private void updateCache() {
        lastKP = Preferences.getDouble("Turret/kP", ShootingConstants.TURRET_P);
        lastKD = Preferences.getDouble("Turret/kD", ShootingConstants.TURRET_D);
        lastKV = Preferences.getDouble("Turret/kV", ShootingConstants.TURRET_KV);
        lastKS = Preferences.getDouble("Turret/kS", ShootingConstants.TURRET_KS);
    }

    private boolean preferencesChanged() {
        return lastKP != Preferences.getDouble("Turret/kP", ShootingConstants.TURRET_P)
                || lastKD != Preferences.getDouble("Turret/kD", ShootingConstants.TURRET_D)
                || lastKV != Preferences.getDouble("Turret/kV", ShootingConstants.TURRET_KV)
                || lastKS != Preferences.getDouble("Turret/kS", ShootingConstants.TURRET_KS);
    }

    public void zeroTurretEncoder() {
        turretMotor.getEncoder().setPosition(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES));
        turretMotor.configure(turretMotorConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    public void zeroTurret() {
        turretMotor.getEncoder().setPosition(ShootingConstants.TURRET_ZERO_OFFSET_RADIANS);
        turretPID.reset(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES));
        turretMotorConfig.apply(turretSoftLimits);
        turretMotor.configure(turretMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        isZeroed = true;
    }

    public boolean getLimitSwitch() {
        return turretSwitch.get();
    }

    public boolean isLeftOfKicker() {
        leftOfKicker = !(turretMotor.getEncoder().getPosition() > Units.degreesToRadians(157.25));
        return leftOfKicker;
    }

    public void setTurret(double angle) {
        if (!isZeroed) {
            this.setVoltage(0.0);
        } else {
            double currentAngle = turretMotor.getEncoder().getPosition();

            double pidOutput = turretPID.calculate(currentAngle, getTurretSetpoint(angle, currentAngle));

            var setpoint = turretPID.getSetpoint();

            double ffOutput = turretFF.calculate(setpoint.velocity);

            turretMotor.setVoltage(pidOutput + ffOutput);
        }

        if (turretPID.getPositionError() < Units.degreesToRadians(0.5)) snappingBack = false;
    }

    public double getTurretSetpoint(double targetAngle, double currentAngle) {
        double delta = MathUtil.angleModulus(targetAngle - currentAngle);

        double setpointRadians = currentAngle + delta;

        if (setpointRadians > Units.degreesToRadians(ShootingConstants.TURRET_FORWARD_LIMIT_DEGREES - 0.5)) {
            setpointRadians -= 2 * Math.PI;
            snappingBack = true;
        } else if (setpointRadians < Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES + 0.5)) {
            setpointRadians += 2 * Math.PI;
            snappingBack = true;
        }

        return setpointRadians;
    }

    public void setSnapBackState(boolean state) {
        snappingBack = state;
    }

    public boolean getSnapBackState() {
        return snappingBack;
    }

    public void setVoltage(double voltage) {
        turretMotor.setVoltage(voltage);
    }

    public void valorantFlick() {
        turretPID.reset(turretMotor.getAbsoluteEncoder().getPosition());
    }

    public void resetPID() {
        turretPID.reset(turretMotor.getEncoder().getPosition());
    }

    private void updatePreferences() {
        turretPID.setP(Preferences.getDouble("Turret/kP", ShootingConstants.TURRET_P));
        turretPID.setD(Preferences.getDouble("Turret/kD", ShootingConstants.TURRET_D));
        turretFF.setKs(Preferences.getDouble("Turret/kS", ShootingConstants.TURRET_KS));
        turretFF.setKv(Preferences.getDouble("Turret/kV", ShootingConstants.TURRET_KV));
    }



    @Override
    public void periodic() {

        if (preferencesChanged()) {
            updatePreferences();
        }

        Logger.recordOutput("Turret/turret angle", turretMotor.getEncoder().getPosition());
        Logger.recordOutput("Turret/turret error", turretPID.getPositionError());
        Logger.recordOutput("Turret/setpoint", turretPID.getSetpoint());
        Logger.recordOutput("Turret/turret velocity", turretMotor.getEncoder().getVelocity());
        Logger.recordOutput("Turret/limit switch", turretSwitch.get());
    }

    private void createAngleRPMToScalarMap() {
        AngleToRPMScalarMap.put(Units.degreesToRadians(-45.0), 0.938);
        AngleToRPMScalarMap.put(Units.degreesToRadians(0.0), 1.0);
        AngleToRPMScalarMap.put(Units.degreesToRadians(22.5), 1.0);
        AngleToRPMScalarMap.put(Units.degreesToRadians(45.0), 0.938);
    }

    public double getAngleRPMToScalarMap(double angle) {
        return AngleToRPMScalarMap.get(angle);
    }

}
