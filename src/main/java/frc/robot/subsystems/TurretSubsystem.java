package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import dev.doglog.internal.tunable.Tunable;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;

public class TurretSubsystem extends SubsystemBase {
    private ProfiledPIDController turretPID = new ProfiledPIDController(ShootingConstants.TURRET_P.get(), 0, ShootingConstants.TURRET_D.get(), ShootingConstants.TURRET_CONSTRAINTS);
    private SparkFlex turretMotor = new SparkFlex(ShootingConstants.TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private NetworkTablesUtils turretNT = NetworkTablesUtils.getTable("Turret");
    private final SimpleMotorFeedforward turretFF = new SimpleMotorFeedforward(
            ShootingConstants.TURRET_KS.get(),
            ShootingConstants.TURRET_KV.get(),
            ShootingConstants.TURRET_KA
    );

    public TurretSubsystem() {
        SparkFlexConfig turretMotorConfig = new SparkFlexConfig();
        turretMotorConfig.inverted(false);
        turretMotorConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        turretMotor.configure(turretMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setTurret(double angle) {
        double currentAngle = MathUtils.turretEncoderToRadians(
                turretMotor.getAbsoluteEncoder().getPosition());

        // Run profiled PID
        double pidOutput = turretPID.calculate(currentAngle, angle);

        // Get motion profile setpoint
        var setpoint = turretPID.getSetpoint();

        // Feedforward uses desired velocity
        double ffOutput = turretFF.calculate(setpoint.velocity);

        turretMotor.setVoltage(pidOutput + ffOutput);
    }
    
    public static double getClosestTurretAngle(double targetAngle, Rotation2d angle, double forwardLimit, double reverseLimit) {
        double currentTotalRadians = (angle.getRotations() * 2 * Math.PI);
        double closestOffset = Units.degreesToRadians(targetAngle) - angle.getRadians();
        if (closestOffset > Math.PI) {
            closestOffset -= 2 * Math.PI;
        } else if (closestOffset < -Math.PI) {
            closestOffset += 2 * Math.PI;
        }

        double finalOffset = currentTotalRadians - closestOffset;
        if ((currentTotalRadians + closestOffset) % (2 * Math.PI) == (currentTotalRadians - closestOffset) % (2 * Math.PI)) {
            if (finalOffset > 0) {
                finalOffset = currentTotalRadians - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalRadians + Math.abs(closestOffset);
            }
        }
        if (finalOffset > Units.degreesToRadians(forwardLimit)) {
            finalOffset -= (2 * Math.PI);
        } else if (finalOffset < Units.degreesToRadians(reverseLimit)) {
            finalOffset += (2 * Math.PI);
        }

        return Units.radiansToDegrees(finalOffset);
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

        turretNT.setEntry("turret angle", MathUtils.turretEncoderToRadians(turretMotor.getAbsoluteEncoder().getPosition()));
        turretNT.setEntry("turret error", turretPID.getPositionError());
    }

}
