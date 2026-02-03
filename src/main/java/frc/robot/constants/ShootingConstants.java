package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.TunableNumber;

public class ShootingConstants {

    public static final int TOP_SHOOTER_MOTOR_ID = 17;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 16;
    public static final int INDEXER_MOTOR_ID = 22;
    public static final int HOOD_ACTUATOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 14;
    public static final int TURRET_LINEBREAK_ID = 2;
    public static final double TURRET_GEAR_RATIO = 0.020202; // 1 : 45
    public static final double TURRET_ENCODER_TO_RADIANS_CONVERSION_FACTOR = 2 * Math.PI * TURRET_GEAR_RATIO;
    public static final double TURRET_PINION_CIRCUMFERENCE = 0.0958;
    public static final double  TURRET_SPUR_GEAR_RADIUS = 0.1295;
    public static final double TURRET_FORWARD_LIMIT_DEGREES = 400;
    public static final double TURRET_REVERSE_LIMIT_DEGREES = 0;

    public static final double SHOOTER_VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60;

    public static final TunableNumber SHOOTER_KS = new TunableNumber("shooter_ks", 0.09);
    public static final TunableNumber SHOOTER_KV = new TunableNumber("shooter_kv", 0.0165);
    public static final double SHOOTER_KA = 0.0;

    public static final TunableNumber SHOOTER_P = new TunableNumber("shooter_p", 0.0);
    public static final double SHOOTER_I = 0.0;
    public static final TunableNumber SHOOTER_D = new TunableNumber("shooter_d", 0.0);

    public static final double HOOD_MAX_ANGLE_DEGREES = 90.0;
    public static final double HOOD_MIN_ANGLE_DEGREES = 0.0;

    // inches
    public static final Translation2d TURRET_OFFSET = new Translation2d(Units.inchesToMeters(0.0), Units.inchesToMeters(-15.0));

    public static TunableNumber TURRET_P = new TunableNumber("turret_p", 8.0);
    public static TunableNumber TURRET_D = new TunableNumber("turret_d", 0.0);
    public static final TrapezoidProfile.Constraints TURRET_CONSTRAINTS = new TrapezoidProfile.Constraints(8 * Math.PI, 32 * Math.PI);

    public static TunableNumber TURRET_KS = new TunableNumber("turret_ks", 0.0);
    public static TunableNumber TURRET_KV = new TunableNumber("turret_kv", 0.0);
    public static double TURRET_KA = 0.0;

    public static final double HOOD_MOTOR_GEAR_RATIO = 0.0;

    public static final double TURRET_MOTOR_GEAR_RATIO = 0.0;

    // how many ticks per motor rotation
    public static double HOOD_ENCODER_RATIO = 0.0;

    public static double TURRET_ENCODER_RATIO = 0.0;

    public static TunableNumber TOP_SHOOTER_RPM = new TunableNumber("top_shooter_rpm", 2500);
    public static TunableNumber BOTTOM_SHOOTER_RPM = new TunableNumber("bottom_shooter_rpm", 6500);
    public static final double AUTO_TOP_SHOOTER_RPM = 2500;
    public static final double AUTO_BOTTOM_SHOOTER_RPM = 6500;

    // Just make it spun up the entire time unc
    public static final double AUTO_SHOOTER_TIMEOUT_SECONDS = 2.0;

}