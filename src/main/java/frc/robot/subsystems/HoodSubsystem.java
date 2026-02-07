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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;

import static frc.robot.Constants.TARGET_POSE_ROTATION;

public class HoodSubsystem extends SubsystemBase {

    private final LinearServo hoodActuator = new LinearServo(ShootingConstants.HOOD_ACTUATOR_ID, 50, 0);

    private final InterpolatingDoubleTreeMap distanceToHoodSetpointMap =
            new InterpolatingDoubleTreeMap();

    public HoodSubsystem() {
        createHoodSetpointMap();
    }

    /**
     * Set hood angle
     * @param angle The angle in degrees to set the hood.
     */

    public void setAngle(double angle) {
        double percentage = (angle - ShootingConstants.HOOD_MIN_ANGLE_DEGREES) /
                (ShootingConstants.HOOD_MAX_ANGLE_DEGREES - ShootingConstants.HOOD_MIN_ANGLE_DEGREES);

        hoodActuator.setPosition(percentage);
    }

    /**
     *
     * @param distance distance in meters from target
     */

    public void setHoodWithDistance(double distance) {
        setAngle(distanceToHoodSetpointMap.get(distance));
    }

    private void createHoodSetpointMap() {
        // distance (m), setpoint (deg)
        // measure in 8" increments or smth
        distanceToHoodSetpointMap.put(0.0, 0.0);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/hood position", hoodActuator.getPosition());
        SmartDashboard.putNumber("Hood/hood angle", hoodActuator.getPosition() * (ShootingConstants.HOOD_MAX_ANGLE_DEGREES - ShootingConstants.HOOD_MIN_ANGLE_DEGREES));
    }

}
