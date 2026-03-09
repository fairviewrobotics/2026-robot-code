package frc.robot.constants;

public class IntakeConstants {
    public static final int INTAKE_DEPLOY_MOTOR_ID = 18;
    public static final int INTAKE_ROLLER_MOTOR_ID = 21;

    // Empirically determined
    public static final double INTAKE_TUNED_RPM = 3200;

    public static final double INTAKE_DEPLOY_P = 0.1;
    public static final double INTAKE_DEPLOY_D = 0.0;


    public static final double INTAKE_DEPLOY_MOTOR_VOLTAGE = 2.0;

    // 50 T, 20 DP
//    public static final double INTAKE_DEPLOY_MOTOR_CONVERSION_FACTOR = (Math.PI * 2.5) / 9.0;

    public static final int DEPLOYED_CURRENT_LIMIT = 10;

    public static final double MAX_RPM_VORTEX = 6500;
}