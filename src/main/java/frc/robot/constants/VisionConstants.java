package frc.robot.constants;

import frc.robot.utils.TunableNumber;

public class VisionConstants {

    public static TunableNumber BASE_VISION_XY_STD_DEV = new TunableNumber("base-vision-xy-std-dev", 0.02);
    public static TunableNumber BASE_VISION_THETA_STD_DEV = new TunableNumber("base-vision-theta-std-dev", 0.06);

    public static TunableNumber ODOMETRY_XY_STD_DEV = new TunableNumber("odometry-xy-std-dev", 0.05);
    public static TunableNumber ODOMETRY_THETA_STD_DEV = new TunableNumber("odometry-theta-std-dev", 0.05);

    public static TunableNumber SINGLE_TAG_DISTRUST_COEFFICIENT = new TunableNumber("single-tag-distrust-coefficient", 2.0);

    // Meters
    public static final double MAX_ACCEPTABLE_TAG_RANGE = 3.0;
    public static final double MAX_Z_ERROR = 0.75;

    public static final double MAX_POSE_AMBIGUITY = 0.15;

    // Inches

    public static final double SHOOTER_CAM_POSE_X = 14;
    public static final double SHOOTER_CAM_POSE_Y = 7.5;
    public static final double SHOOTER_CAM_POSE_Z = 4.5;


    // Degrees

    public static final double SHOOTER_CAM_POSE_ROLL = 0.0;
    public static final double SHOOTER_CAM_POSE_PITCH = 41;
    public static final double SHOOTER_CAM_POSE_YAW = 0.0;

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

    public static final double BALL_CAM_POSE_X = 0.0;
    public static final double BALL_CAM_POSE_Y = 0.0;
    public static final double BALL_CAM_POSE_Z = 0.0;


    // Degrees

    public static final double BALL_CAM_POSE_ROLL = 0.0;
    public static final double BALL_CAM_POSE_PITCH = 0.0;
    public static final double BALL_CAM_POSE_YAW = 0.0;

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



}
