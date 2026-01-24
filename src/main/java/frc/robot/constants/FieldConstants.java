package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

    public static final double FIELD_BORDER_MARGIN_METERS = 0.5;
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.21);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);

    public static final double BALL_HEIGHT_METERS = Units.inchesToMeters(6);

    // Garage bullshit values

    public static final Pose3d RED_HUB_POSE3D = new Pose3d(4.62534,4.034663,1.822, new Rotation3d(0,0,0));
    public static final Pose3d BLUE_HUB_POSE3D = new Pose3d(4.62534 + 7.2898,4.034663,1.822, new Rotation3d(0,0,0));
    public static final Pose2d BOTTOM_LEFT_CORNER_DEPOT = new Pose2d(0, 4.924044, new Rotation2d(0,0));
    public static final Pose2d BOTTOM_RIGHT_CORNER_DEPOT = new Pose2d(0.6858,4.924044, new Rotation2d(0,0));
    public static final Pose2d TOP_LEFT_CORNER_DEPOT = new Pose2d(0,7.002526, new Rotation2d(0,0));
    public static final Pose2d TOP_RIGHT_CORNER_DEPOT = new Pose2d(0.6858,7.002526, new Rotation2d(0,0));
    public static final Pose2d FEEDER_STATION = new Pose2d(0,0.632968, new Rotation2d(0,0));
    // 1 is center face from drive POV then go clockwise w/ driver POV

    // Should just be ID 10, 5, 2 pose

    public static final Pose2d RED_HUB_CENTER_POINT = new Pose2d(0, 0, Rotation2d.kZero);
    public static final Pose2d BLUE_HUB_CENTER_POINT = new Pose2d(0, 0, Rotation2d.kZero);



}
