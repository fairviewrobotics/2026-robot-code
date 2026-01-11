package frc.robot.constants;

import frc.robot.utils.ConfigManager;
import frc.robot.utils.TunableNumber;

public class ShooterConstants {
    //TODO: Update constants and tune values
    public static final int TOP_SHOOTER_MOTOR_ID = 17;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 16;
    public static final int INDEXER_MOTOR_ID = 22;

    public static final double SHOOTER_KS = 0.09;
    public static final double SHOOTER_KV = 0.0165;
    public static final double SHOOTER_KA = 0;

    public static final double SHOOTER_P = 0;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;

    public static TunableNumber TOP_SHOOTER_RPM = new TunableNumber("top_shooter_rpm", 2500);
    public static TunableNumber BOTTOM_SHOOTER_RPM = new TunableNumber("bottom_shooter_rpm", 6500);
    public static final double AUTO_TOP_SHOOTER_RPM = 2500;
    public static final double AUTO_BOTTOM_SHOOTER_RPM = 6500;

    // Just make it spun up the entire time unc
    public static final double AUTO_SHOOTER_TIMEOUT_SECONDS = 2.0;

}