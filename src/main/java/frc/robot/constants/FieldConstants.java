package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

    public static final double FIELD_BORDER_MARGIN_METERS = 0.5;
    public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.21);
    public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);

    public static final double BALL_HEIGHT_METERS = Units.inchesToMeters(6);

    public static final Pose2d STARTING_ODOMETRY_POSE = new Pose2d(3, 3, Rotation2d.fromDegrees(0));

    public static final Pose2d DEPOT_INTAKE_POSE = new Pose2d(2.5,3.75,new Rotation2d(0)); // todo find the correct x and y
    public static final Pose2d AUTO_SHOOT_POSE = new Pose2d(2.28,3.23,new Rotation2d(0)); // todo please chose new x and y

    // 1 is center face from drive POV then go clockwise w/ driver POV

    // Should just be ID 10, 5, 2 pose

    public static final Pose2d RED_HUB_CENTER_POINT = new Pose2d(0, 0, Rotation2d.kZero);
    public static final Pose2d BLUE_HUB_CENTER_POINT = new Pose2d(0, 0, Rotation2d.kZero);



}