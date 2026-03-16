package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.constants.FieldConstants;

public class FlipOverYUtil {
    public static Pose2d apply(Pose2d pose){
        return new Pose2d( pose.getX(), FieldConstants.FIELD_WIDTH_METERS - pose.getY(), pose.getRotation());
    }
}
