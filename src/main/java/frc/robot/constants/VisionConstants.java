package frc.robot.constants;

import frc.robot.utils.TunableNumber;

public class VisionConstants {

    public static double BASE_VISION_XY_STD_DEV = 0.02;
    public static double BASE_VISION_THETA_STD_DEV = 0.06;

    public static double SINGLE_TAG_DISTRUST_COEFFICIENT = 2.0;

    // Meters
    public static final double MAX_ACCEPTABLE_TAG_RANGE = 3.0;
    public static final double MAX_Z_ERROR = 0.75;

    // Percentage
    public static final double TAG_AMBIGUITY_TOLERANCE = 0.15;

    // Inches
    public static final double BACK_CAM_POSE_X = -13.75;
    public static final double BACK_CAM_POSE_Y = -9.25;
    public static final double BACK_CAM_POSE_Z = 17.0;

    // Degrees
    public static final double BACK_CAM_POSE_ROLL = 0.0;
    public static final double BACK_CAM_POSE_PITCH = -17;
    public static final double BACK_CAM_POSE_YAW = 180.0;

    public static TunableNumber SHOOTER_CAM_ADJUST_X =
            new TunableNumber(
                    "shooter_cam_adjust_x",
                    0.0);

    public static TunableNumber SHOOTER_CAM_ADJUST_Y =
            new TunableNumber(
                    "shooter_cam_adjust_y",
                    0.0);

    public static TunableNumber SHOOTER_CAM_ADJUST_Z =
            new TunableNumber(
                    "shooter_cam_adjust_z",
                    0.0);


    public static TunableNumber SHOOTER_CAM_ADJUST_ROLL =
            new TunableNumber(
                    "shooter_cam_adjust_roll",
                    0.0);

    public static TunableNumber SHOOTER_CAM_ADJUST_PITCH =
            new TunableNumber(
                    "shooter_cam_adjust_pitch",
                    0.0);

    public static TunableNumber SHOOTER_CAM_ADJUST_YAW =
            new TunableNumber(
                    "shooter_cam_adjust_yaw",
                    0.0);

    // Inches

    public static final double BALL_CAM_POSE_X = -15.0;
    public static final double BALL_CAM_POSE_Y = 0.0;
    public static final double BALL_CAM_POSE_Z = 4.25;


    // Degrees

    public static final double BALL_CAM_POSE_ROLL = 0.0;
    public static final double BALL_CAM_POSE_PITCH = 0.0;
    public static final double BALL_CAM_POSE_YAW = 180.0;

    public static TunableNumber BALL_CAM_ADJUST_X =
            new TunableNumber(
                    "ball_cam_adjust_x",
                    0.0);

    public static TunableNumber BALL_CAM_ADJUST_Y =
            new TunableNumber(
                    "ball_cam_adjust_y",
                    0.0);

    public static TunableNumber BALL_CAM_ADJUST_Z =
            new TunableNumber(
                    "ball_cam_adjust_z",
                    0.0);


    public static TunableNumber BALL_CAM_ADJUST_ROLL =
            new TunableNumber(
                    "ball_cam_adjust_roll",
                    0.0);

    public static TunableNumber BALL_CAM_ADJUST_PITCH =
            new TunableNumber(
                    "ball_cam_adjust_pitch",
                    0.0);

    public static TunableNumber BALL_CAM_ADJUST_YAW =
            new TunableNumber(
                    "ball_cam_adjust_yaw",
                    0.0);

    public static TunableNumber BUMPER_DETECTION_RETRACT_TY_DEGREES =
            new TunableNumber(
                    "bumper-detection-retract-ty-degrees",
                    0.0);

}
