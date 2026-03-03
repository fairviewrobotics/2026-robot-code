package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.AllianceFlipUtil;

public class HoodSubsystem extends SubsystemBase {

    private double targetAngle = ShootingConstants.HOOD_MIN_ANGLE_DEGREES;
    private final LinearServo hoodActuator = new LinearServo(ShootingConstants.HOOD_ACTUATOR_ID, 50, 0);
    private final SwerveSubsystem swerveSubsystem;
    private final InterpolatingDoubleTreeMap distanceToHoodSetpointMap =
            new InterpolatingDoubleTreeMap();

    public HoodSubsystem(SwerveSubsystem swerveSubsystem) {
        createHoodSetpointMap();
        initializePreferences();
        this.swerveSubsystem = swerveSubsystem;
    }

    public void initializePreferences() {
        Preferences.initDouble("Hood/HOOD_SETPOINT", 0.05);
    }

    /**
     *
     * @param distance distance in meters from target
     */

    public void setHoodWithDistance(double distance) {
        setHood(distanceToHoodSetpointMap.get(distance));
    }

    private void createHoodSetpointMap() {
        // distance (m), setpoint (deg)
        // measure in 8" increments or smth
        distanceToHoodSetpointMap.put(0.0, 0.0);
    }

    public double getHoodSetpoint(double distance) {return distanceToHoodSetpointMap.get(distance);}

    @Override
    public void periodic() {
        ChassisSpeeds speeds = swerveSubsystem.getRobotVelocity();

        // 1.25s for hood to go from fully extended to retracted
        double lookaheadTime = 1.0;

        Pose2d futurePose = swerveSubsystem.getPose().exp(
                new Twist2d(
                        speeds.vxMetersPerSecond * lookaheadTime,
                        speeds.vyMetersPerSecond * lookaheadTime,
                        speeds.omegaRadiansPerSecond * lookaheadTime
                )
        );

        boolean isInvadingZone =
                AllianceFlipUtil.apply(FieldConstants.TRENCH_BOUNDS).contains(futurePose.getTranslation())
                || AllianceFlipUtil.apply(FieldConstants.TRENCH_BOUNDS).contains(swerveSubsystem.getPose().getTranslation());

        double finalSetpoint = isInvadingZone ? ShootingConstants.HOOD_MIN_ANGLE_DEGREES : targetAngle;

        // setHood(finalSetpoint);
    }

    public void setHood(double angle) {
        double percentage = (angle - ShootingConstants.HOOD_MIN_ANGLE_DEGREES) /
                (ShootingConstants.HOOD_MAX_ANGLE_DEGREES - ShootingConstants.HOOD_MIN_ANGLE_DEGREES);
        hoodActuator.setClampedPosition(percentage);
    }

    // Delete later
    public void setHoodWithPreferences() {
        hoodActuator.setClampedPosition(Preferences.getDouble("Hood/HOOD_SETPOINT", 0.05));
    }

}


