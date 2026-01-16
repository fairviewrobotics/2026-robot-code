package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;

import static frc.robot.Constants.TARGET_POSE_ROTATION;

public class HoodSubsystem extends SubsystemBase {

    private final SparkFlex hoodMotor = new SparkFlex(ShootingConstants.TOP_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private NetworkTablesUtils hoodNT = NetworkTablesUtils.getTable("Hood");

    private final ProfiledPIDController hoodPID = new ProfiledPIDController(
            ShootingConstants.HOOD_P.get(),
            0.0,
            ShootingConstants.HOOD_D.get(),
            ShootingConstants.HOOD_CONSTRAINTS
    );

    private final InterpolatingDoubleTreeMap distanceToHoodSetpointMap =
            new InterpolatingDoubleTreeMap();

    public HoodSubsystem() {

        SparkFlexConfig hoodMotorConfig = new SparkFlexConfig();

        hoodMotorConfig
                .inverted(true);
        hoodMotorConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

        hoodMotor.configure(hoodMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        createHoodSetpointMap();
    }

    public void setHood(double angle) {
        hoodMotor.setVoltage(hoodPID.calculate(MathUtils.hoodEncoderToRadians(hoodMotor.getAbsoluteEncoder().getPosition()), angle));
    }

    public void setHoodWithDistance(double distance) {
        setHood(distanceToHoodSetpointMap.get(distance));
    }

    private void createHoodSetpointMap() {
        // distance (m), setpoint (rad)
        // measure in 8" increments or smth
        distanceToHoodSetpointMap.put(0.0, 0.0);

    }

    @Override
    public void periodic() {

        TunableNumber.ifChanged(
                hashCode(),
                () -> {
                    hoodPID.setP(ShootingConstants.HOOD_P.get());
                    hoodPID.setD(ShootingConstants.HOOD_D.get());
                },
                ShootingConstants.HOOD_P,
                ShootingConstants.HOOD_D
        );

        hoodNT.setEntry("hood angle", MathUtils.hoodEncoderToRadians(hoodMotor.getAbsoluteEncoder().getPosition()));
        hoodNT.setEntry("hood error", hoodPID.getPositionError());

    }

}
