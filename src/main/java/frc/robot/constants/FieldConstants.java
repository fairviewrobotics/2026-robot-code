package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.Bounds;

public class FieldConstants {

    public static final double FIELD_BORDER_MARGIN_METERS = 0.5;
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.069326;

    public static final double BALL_HEIGHT_METERS = Units.inchesToMeters(6.0);

    public static final Pose3d BLUE_HUB_POSE3D = new Pose3d(4.62534,4.034663,1.822, Rotation3d.kZero);
    public static final Pose3d RED_HUB_POSE3D = new Pose3d(4.62534 + 7.2898,4.034663,1.822, Rotation3d.kZero);
    public static final Pose2d BLUE_PASS_RIGHT_POSE = new Pose2d(0.6, 7.4, Rotation2d.kZero);
    public static final Pose2d BLUE_PASS_LEFT_POSE = new Pose2d(0.6, 0.6, Rotation2d.kZero);

    public static final Pose2d BLUE_TRENCH_LEFT = new Pose2d(4.511, 7.415, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_RIGHT = new Pose2d(4.511, 0.640, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS = new Pose2d(6.0, 7.415, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_LEFT_TRANSITION_PICKUP = new Pose2d(7.657, 7.415, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_TRENCH_LEFT_TRANSITION_DROPOFF = new Pose2d(7.657, 7.415, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_LEFT_PICKUP_END = new Pose2d(7.805, 4.611, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_AUTO_SHOOT_LEFT_POINT = new Pose2d(3.064, 6.5, Rotation2d.fromDegrees(-240.0));
    public static final Pose2d BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION = new Pose2d(3.0, 7.415, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_RIGHT_TO_SHOOT_TRANSITION = new Pose2d(3.0, 0.640, Rotation2d.kZero);
    public static final Pose2d BLUE_AUTO_SHOOT_RIGHT_POINT = new Pose2d(3.064, 1.55, Rotation2d.fromDegrees(-125));

    public static final Pose2d START_POINT = new Pose2d(0.783, 1.79, Rotation2d.kZero);
    public static final Pose2d TOP_RIGHT_POINT = new Pose2d(1.783,1.79, Rotation2d.kZero);
    public static final Pose2d BOTTOM_RIGHT_POINT = new Pose2d(1.783,0.79, Rotation2d.kZero);
    public static final Pose2d BOTTOM_LEFT_POINT = new Pose2d(0.783,0.79, Rotation2d.kZero);
    public static final Pose2d CARPET_POINT = new Pose2d(3.8, 3.8, Rotation2d.kCCW_90deg);
    public static final Pose2d CARPET_POINT2 = new Pose2d(3.8, 3.8, Rotation2d.k180deg);
    public static final Pose2d ODOMETRY_RESET_POINT = new Pose2d(3, 3, Rotation2d.kPi);

    public static final Bounds TRENCH_BOUNDS = new Bounds(4, 5.25, 0, 8.5);

}
