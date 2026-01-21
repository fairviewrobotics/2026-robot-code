package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

    // Locations for the swerve drive modules relative to the robot center.
    private static final Translation2d FRONT_LEFT = new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(15));
    private static final Translation2d FRONT_RIGHT = new Translation2d(Units.inchesToMeters(15), -Units.inchesToMeters(15));
    private static final Translation2d REAR_LEFT = new Translation2d(-Units.inchesToMeters(15), Units.inchesToMeters(15));
    private static final Translation2d REAR_RIGHT = new Translation2d(-Units.inchesToMeters(15), -Units.inchesToMeters(15));

    // Creating my kinematics object using the module locations
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    );

    public static final double WHEEL_DIAMETER_INCHES = 4.0;

    public static final double MAX_SPEED_MPS = 4.0;
    public static final double MAX_ACCELERATION = 1.0;

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double TURN_GEAR_RATIO = 21.4285714286;
}
