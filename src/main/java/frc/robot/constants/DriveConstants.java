package frc.robot.constants;

import dev.doglog.internal.tunable.Tunable;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.SwerveModuleConfig;
import frc.robot.utils.TunableNumber;

public class DriveConstants {

    public static TunableNumber CONTROLLER_DEADBAND = new TunableNumber("controller-deadband", 0.2);
    public static final double ODOMETRY_FREQUENCY = 20; //Hz

    public static final double WHEEL_DIAMETER_INCHES = 3.0;

    public static final double MAX_SPEED_MPS = 4.0;
    public static final double MAX_ACCELERATION = 1.0;

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double ANGLE_GEAR_RATIO = 46.42;

    public static final double DRIVE_POSITION_CONVERSION_FACTOR = Math.PI * Units.inchesToMeters(WHEEL_DIAMETER_INCHES) / DRIVE_GEAR_RATIO;
    public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = Math.PI * Units.inchesToMeters(WHEEL_DIAMETER_INCHES) / DRIVE_GEAR_RATIO / 60;

    public static TunableNumber MAX_XY_SPEED_MPS = new TunableNumber("max-module-speed", 3.0);
    public static TunableNumber MAX_THETA_SPEED_RAD_PS = new TunableNumber("max-module-theta", Math.PI);

    public static TunableNumber DRIVE_P = new TunableNumber("drive-p", 0.0020645);
    public static TunableNumber DRIVE_D = new TunableNumber("drive-d", 0.0);
    public static TunableNumber DRIVE_KS = new TunableNumber("drive-ks", 0.0);
    public static TunableNumber DRIVE_KV = new TunableNumber("drive-kv", 0.0);
    public static TunableNumber DRIVE_KA = new TunableNumber("drive-ka", 0.0);
    public static TunableNumber ANGLE_P = new TunableNumber("angle-p", 0.01);
    public static TunableNumber ANGLE_D = new TunableNumber("angle-d", 0.0);

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 7;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 8;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 3;
    public static final int REAR_LEFT_DRIVE_MOTOR_ID = 5;
    public static final int REAR_LEFT_ANGLE_MOTOR_ID = 6;
    public static final int REAR_RIGHT_DRIVE_MOTOR_ID = 4;
    public static final int REAR_RIGHT_ANGLE_MOTOR_ID = 5;

    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_LEFT_ANGLE_INVERTED = false;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    public static final boolean FRONT_RIGHT_ANGLE_INVERTED = false;
    public static final boolean REAR_LEFT_DRIVE_INVERTED = false;
    public static final boolean REAR_LEFT_ANGLE_INVERTED = false;
    public static final boolean REAR_RIGHT_DRIVE_INVERTED = false;
    public static final boolean REAR_RIGHT_ANGLE_INVERTED = false;

    // relative to wpi y
    public static final double DRIVEBASE_WIDTH_INCHES = 30.0;

    // relative to wpi x
    public static final double DRIVEBASE_LENGTH_INCHES = 30.0;

    // Locations for the swerve drive modules relative to the robot center.
    private static final Translation2d FRONT_LEFT = new Translation2d(Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2), Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2));
    private static final Translation2d FRONT_RIGHT = new Translation2d(Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2), -Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2));
    private static final Translation2d REAR_LEFT = new Translation2d(-Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2), Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2));
    private static final Translation2d REAR_RIGHT = new Translation2d(-Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2), -Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2));

    // Creating my kinematics object using the module locations
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT
    );

    public static final TunableNumber[] ANGLE_OFFSETS = {
            new TunableNumber("Offset/Module 0", 38.267), //FL
            new TunableNumber("Offset/Module 1", 160.495), //FR
            new TunableNumber("Offset/Module 2", 288.346), //RL
            new TunableNumber("Offset/Module 3", 159.085) // RR
    };

    public static final SwerveModuleConfig FRONT_LEFT_CONFIG =
            new SwerveModuleConfig(
                    0,
                    FRONT_LEFT_DRIVE_MOTOR_ID,
                    FRONT_LEFT_ANGLE_MOTOR_ID,
                    new Translation2d(
                            Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2),
                            Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2)
                    ),
                    38.267,
                    FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_ANGLE_INVERTED
                    );

    public static final SwerveModuleConfig FRONT_RIGHT_CONFIG =
            new SwerveModuleConfig(
                    1,
                    FRONT_RIGHT_DRIVE_MOTOR_ID,
                    FRONT_RIGHT_ANGLE_MOTOR_ID,
                    new Translation2d(
                            Units.inchesToMeters(DRIVEBASE_WIDTH_INCHES/2),
                            Units.inchesToMeters(-DRIVEBASE_LENGTH_INCHES/2)
                    ),
                    160.495,
                    FRONT_RIGHT_DRIVE_INVERTED,
                    FRONT_RIGHT_ANGLE_INVERTED
            );

    public static final SwerveModuleConfig REAR_LEFT_CONFIG =
            new SwerveModuleConfig(
                    2,
                    REAR_LEFT_DRIVE_MOTOR_ID,
                    REAR_LEFT_ANGLE_MOTOR_ID,
                    new Translation2d(
                            Units.inchesToMeters(-DRIVEBASE_WIDTH_INCHES/2),
                            Units.inchesToMeters(DRIVEBASE_LENGTH_INCHES/2)
                    ),
                    288.346,
                    FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_ANGLE_INVERTED
            );

    public static final SwerveModuleConfig REAR_RIGHT_CONFIG =
            new SwerveModuleConfig(
                    3,
                    REAR_LEFT_DRIVE_MOTOR_ID,
                    REAR_LEFT_ANGLE_MOTOR_ID,
                    new Translation2d(
                            Units.inchesToMeters(-DRIVEBASE_WIDTH_INCHES/2),
                            Units.inchesToMeters(-DRIVEBASE_LENGTH_INCHES/2)
                    ),
                    159.085,
                    FRONT_LEFT_DRIVE_INVERTED,
                    FRONT_LEFT_ANGLE_INVERTED
            );

}
