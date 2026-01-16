package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.TunableNumber;

public class ShootingConstants {
    //TODO: Update constants and tune values
    public static final int TOP_SHOOTER_MOTOR_ID = 17;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 16;
    public static final int INDEXER_MOTOR_ID = 22;
    public static final int HOOD_MOTOR_ID = 0;
    public static final int TURRET_MOTOR_ID = 0;

    public static final double SHOOTER_KS = 0.09;
    public static final double SHOOTER_KV = 0.0165;
    public static final double SHOOTER_KA = 0;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    public static TunableNumber HOOD_P = new TunableNumber("hood_p", 0.0);
    public static TunableNumber HOOD_D = new TunableNumber("hood_d", 0.0);
    public static final TrapezoidProfile.Constraints HOOD_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI / 2  ,Math.PI);

    public static TunableNumber TURRET_P = new TunableNumber("turret_p", 0.0);
    public static TunableNumber TURRET_D = new TunableNumber("turret_d", 0.0);
    public static final TrapezoidProfile.Constraints TURRET_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.PI / 2  ,Math.PI);

    public static TunableNumber TURRET_KS = new TunableNumber("turret_ks", 0.0);
    public static TunableNumber TURRET_KV = new TunableNumber("turret_kv", 0.0);
    public static double TURRET_KA = 0.0;

    public static double HOOD_MOTOR_GEAR_RATIO = 0.0;

    public static double TURRET_MOTOR_GEAR_RATIO = 0.0;

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