package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {
    private final ProfiledPIDController turretPID = new ProfiledPIDController(
            ShootingConstants.TURRET_P,
            0.0,
            ShootingConstants.TURRET_D,
            ShootingConstants.TURRET_CONSTRAINTS);
    private boolean isZeroed = false;
    private DigitalInput turretLinebreak = new DigitalInput(ShootingConstants.TURRET_LINEBREAK_ID);
    private final SparkFlex turretMotor = new SparkFlex(ShootingConstants.TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
            ShootingConstants.TURRET_KS,
            ShootingConstants.TURRET_KV,
            ShootingConstants.TURRET_KA
    );

    private double lastKP = ShootingConstants.TURRET_P;
    private double lastKD = ShootingConstants.TURRET_D;
    private double lastKS = ShootingConstants.TURRET_KS;
    private double lastKV = ShootingConstants.TURRET_KV;

    public TurretSubsystem() {

        initializePreferences();

        SparkFlexConfig turretMotorConfig = new SparkFlexConfig();
        SoftLimitConfig turretSoftLimits = new SoftLimitConfig();

        turretSoftLimits
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(Units.degreesToRadians(ShootingConstants.TURRET_FORWARD_LIMIT_DEGREES))
                .reverseSoftLimit(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES));

        turretMotorConfig
                .inverted(false)
                .apply(turretSoftLimits)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .encoder.positionConversionFactor(ShootingConstants.TURRET_ENCODER_TO_RADIANS_CONVERSION_FACTOR);

        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        updateCache();

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
    }

    public void zeroTurret() {
        if (!turretLinebreak.get()) {
            this.setVoltage(-1.0);
        } else {
            turretMotor.setVoltage(0.0);
            turretMotor.getEncoder().setPosition(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES)
            );
            turretPID.reset(Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES));
            isZeroed = true;
        }

    }

    public void setTurret(double angle) {
        double currentAngle = turretMotor.getEncoder().getPosition();

        double pidOutput = turretPID.calculate(currentAngle, getTurretSetpoint(angle, currentAngle));

        var setpoint = turretPID.getSetpoint();

        double ffOutput = turretFF.calculate(setpoint.velocity);

        turretMotor.setVoltage(pidOutput + ffOutput);
    }

    public static double getTurretSetpoint(double targetAngle, double currentAngle) {

        double delta = MathUtil.angleModulus(targetAngle - currentAngle);
        double setpointRadians = currentAngle + delta;

        if (setpointRadians > Units.degreesToRadians(ShootingConstants.TURRET_FORWARD_LIMIT_DEGREES - 0.5)) {
            setpointRadians -= 2 * Math.PI;
        } else if (setpointRadians < Units.degreesToRadians(ShootingConstants.TURRET_REVERSE_LIMIT_DEGREES + 0.5)) {
            setpointRadians += 2 * Math.PI;
        }

        return setpointRadians;
    }

    public void setVoltage(double voltage) {
        turretMotor.set(voltage);
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

    }

}
