package frc.robot.utils;

import edu.wpi.first.math.geometry.*;


public class MathUtils {

    public static double RPMtoRadians(double rpm) {
        return rpm * (2 * Math.PI / 60);
    }

    public static final Pose2d zeroPose = new Pose2d(0, 0, new Rotation2d(0));
    public static final Translation2d zeroTranslation = new Translation2d(0.0, 0.0);

    public static final Pose2d getPoseFromRotation(Rotation2d rotation) {return new Pose2d(zeroTranslation, rotation);}
    public static final Pose2d getPoseFromTranslation(Translation2d translation) {return new Pose2d(translation, Rotation2d.fromDegrees(0));}
    public static final Transform2d getTransform2dFromTranslation(Translation2d translation) {return new Transform2d(translation, Rotation2d.kZero);}
    public static final Pose3d getPose3dFromTransform3d(Transform3d transform) {return new Pose3d(transform.getTranslation(), transform.getRotation());}
    public static final Transform3d getTransform3dFromPose3d(Pose3d pose) {return new Transform3d(pose.getTranslation(), pose.getRotation());}


}


