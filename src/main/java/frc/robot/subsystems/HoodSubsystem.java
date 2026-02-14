package frc.robot.subsystems;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;

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
        hoodActuator.setClampedPosition(percentage);
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

    }

}
