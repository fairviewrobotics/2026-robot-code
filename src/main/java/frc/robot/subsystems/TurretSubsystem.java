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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;

public class TurretSubsystem extends SubsystemBase {
    private final ProfiledPIDController turretPID = new ProfiledPIDController(
            ShootingConstants.TURRET_P.get(),
            0.0,
            ShootingConstants.TURRET_D.get(),
            ShootingConstants.TURRET_CONSTRAINTS);
    private boolean isZeroed = false;
    private DigitalInput turretLinebreak = new DigitalInput(ShootingConstants.TURRET_LINEBREAK_ID);
    private final SparkFlex turretMotor = new SparkFlex(ShootingConstants.TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final NetworkTablesUtils turretNT = NetworkTablesUtils.getTable("Turret");
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
            ShootingConstants.TURRET_KS.get(),
            ShootingConstants.TURRET_KV.get(),
            ShootingConstants.TURRET_KA
    );

    public TurretSubsystem() {

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

    public void resetPID() {
        turretPID.reset(turretMotor.getAbsoluteEncoder().getPosition());
    }

    @Override
    public void periodic() {

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    turretPID.setP(ShootingConstants.TURRET_P.get());
                    turretPID.setD(ShootingConstants.TURRET_D.get());
                },
                ShootingConstants.TURRET_P,
                ShootingConstants.TURRET_D
        );

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    turretFF.setKs(ShootingConstants.TURRET_KS.get());
                    turretFF.setKv(ShootingConstants.TURRET_KV.get());
                },
                ShootingConstants.TURRET_KS,
                ShootingConstants.TURRET_KV
        );

        SmartDashboard.putNumber("Turret/turret angle", turretMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Turret/turret error", turretPID.getPositionError());
        SmartDashboard.putBoolean("Turret/turret linebreak", turretLinebreak.get());
    }

}
