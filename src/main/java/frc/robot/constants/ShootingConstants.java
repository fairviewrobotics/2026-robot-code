package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.TunableNumber;

public class ShootingConstants {

    public static final int LEFT_SHOOTER_MOTOR_ID = 17;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 16;
    public static final int HOOD_ACTUATOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 14;
    public static final double TURRET_GEAR_RATIO = 0.02610966057; // 1 : 38.3
    public static final double TURRET_ENCODER_TO_RADIANS_CONVERSION_FACTOR = 2 * Math.PI * TURRET_GEAR_RATIO;
    public static final double TURRET_FORWARD_LIMIT_DEGREES = 360;
    public static final double TURRET_REVERSE_LIMIT_DEGREES = 0;


    public static final double HOOD_MAX_ANGLE_DEGREES = 60.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 15.0;

    public static double TURRET_P = 8.0;
    public static double TURRET_D = 0.0;
    public static final TrapezoidProfile.Constraints TURRET_CONSTRAINTS = new TrapezoidProfile.Constraints(8 * Math.PI, 32 * Math.PI);

    public static double TURRET_KS = 0.0;
    public static double TURRET_KV = 0.0;
    public static double TURRET_KA = 0.0;

    public static final double HOOD_MOTOR_GEAR_RATIO = 0.0;

    public static final double TURRET_MOTOR_GEAR_RATIO = 0.0;

    // how many ticks per motor rotation
    public static double HOOD_ENCODER_RATIO = 0.0;

    public static double TURRET_ENCODER_RATIO = 0.0;

    public static final double LEFT_SHOOTER_RPM = 2500;
    public static final double RIGHT_SHOOTER_RPM = 6500;

    // Just make it spun up the entire time unc
    public static final double AUTO_SHOOTER_TIMEOUT_SECONDS = 2.0;
    public static final Transform2d TURRET_OFFSET2D =  new Transform2d(0.0, 0.0, Rotation2d.kZero);//TODO: get offset
    public static final Transform3d TURRET_OFFSET3D = new Transform3d(0.0,0.0,0.0, Rotation3d.kZero);//TODO: get offset

}