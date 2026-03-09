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
    public static final double FIELD_WIDTH_METERS = 8.00;

    public static final double BALL_HEIGHT_METERS = Units.inchesToMeters(6.0);

    public static final Pose3d BLUE_HUB_POSE3D = new Pose3d(4.62534,4.034663,1.822, Rotation3d.kZero);
    public static final Pose3d RED_HUB_POSE3D = new Pose3d(4.62534 + 7.2898,4.034663,1.822, Rotation3d.kZero);
    public static final Pose2d BLUE_PASS_RIGHT_POSE = new Pose2d(0.6, 7.4, Rotation2d.kZero);
    public static final Pose2d BLUE_PASS_LEFT_POSE = new Pose2d(0.6, 0.6, Rotation2d.kZero);


    public static final Pose2d BLUE_TRENCH_LEFT = new Pose2d(4.511, 7.415, Rotation2d.kZero);
    public static final Pose2d BLUE_TRENCH_LEFT_TRANSITION_PICKUP = new Pose2d(7.657, 7.415, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_TRENCH_LEFT_PICKUP_END = new Pose2d(7.805, 4.611, Rotation2d.kCW_90deg);
    public static final Pose2d BLUE_AUTO_SHOOT_LEFT_POINT = new Pose2d(3.064, 7.415, new Rotation2d( -Math.PI / 3 - 0.4));

    public static final Pose2d CARPET_POINT = new Pose2d(3.8, 3.8, Rotation2d.kCCW_90deg);
    public static final Pose2d CARPET_POINT2 = new Pose2d(3.8, 3.8, Rotation2d.k180deg);
    public static final Pose2d ODOMETRY_RESET_POINT = new Pose2d(3, 3, Rotation2d.kPi);

    public static final Bounds TRENCH_BOUNDS = new Bounds(4, 5.25, 0, 8.5);

}
