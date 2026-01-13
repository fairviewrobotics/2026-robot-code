package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;

public class HoodSubsystem extends SubsystemBase {

    private final SparkFlex hoodMotor = new SparkFlex(ShootingConstants.TOP_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    NetworkTablesUtils hoodNT = NetworkTablesUtils.getTable("Hood");
    private final ProfiledPIDController hoodPID = new ProfiledPIDController(
            ShootingConstants.HOOD_P.get(),
            0.0,
            ShootingConstants.HOOD_D.get(),
            ShootingConstants.HOOD_CONSTRAINTS
    );

    private final InterpolatingDoubleTreeMap distanceToHoodSetpointMap =
            new InterpolatingDoubleTreeMap();

    public HoodSubsystem() {

        SparkFlexConfig hoodMotor = new SparkFlexConfig();

        hoodMotor
                .inverted(true);
        hoodMotor
                .idleMode(SparkBaseConfig.IdleMode.kBrake);

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
        // measure in 6-10" increments
        distanceToHoodSetpointMap.put(0.0, 0.0);
    }

    @Override
    public void periodic() {
        hoodNT.setEntry("hood angle", MathUtils.hoodEncoderToRadians(hoodMotor.getAbsoluteEncoder().getPosition()));
        hoodNT.setEntry("hood error", hoodPID.getPositionError());
    }

}
