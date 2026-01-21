package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;

import static frc.robot.Constants.TARGET_POSE_ROTATION;

public class HoodSubsystem extends SubsystemBase {

    private final Servo hoodActuator = new Servo(ShootingConstants.HOOD_ACTUATOR_ID);
    private NetworkTablesUtils hoodNT = NetworkTablesUtils.getTable("Hood");

    private final InterpolatingDoubleTreeMap distanceToHoodSetpointMap =
            new InterpolatingDoubleTreeMap();

    public HoodSubsystem() {
        createHoodSetpointMap();
    }

    public void setHood(double angleDegrees) {
        double angleSetpoint = MathUtil.clamp(angleDegrees, ShootingConstants.HOOD_MIN_ANGLE_DEGREES, ShootingConstants.HOOD_MAX_ANGLE_DEGREES);
        hoodActuator.setPosition(Math.toDegrees(angleSetpoint));
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
        hoodNT.setEntry("hood angle", hoodActuator.getAngle());
    }

}
