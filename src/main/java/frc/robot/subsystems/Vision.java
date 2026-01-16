package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.OdometryMeasurement;
import frc.robot.utils.VisionMeasurement;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {

    // This should be 2026 at some point
    // Probably remove IDs 6, 7, 13, 14, 1, 12, 17, 28, 22, 23, 29, 30

    PhotonCamera[] cameras;
    RobotState robotState;
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final Pose3d[] baseCameraPoses =
            new Pose3d[] {
                    // Shooter Cam
                    new Pose3d(
                            Units.inchesToMeters(VisionConstants.SHOOTER_CAM_POSE_X),
                            Units.inchesToMeters(VisionConstants.SHOOTER_CAM_POSE_Y),
                            Units.inchesToMeters(VisionConstants.SHOOTER_CAM_POSE_Z),
                            new Rotation3d(
                                    Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_ROLL),
                                    Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_PITCH),
                                    Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_YAW))),

            };

    private Pose3d[] getAdjustedCameraPoses() {
        return new Pose3d[] {

                new Pose3d(

                        VisionConstants.SHOOTER_CAM_POSE_X + Units.inchesToMeters(VisionConstants.SHOOTER_CAM_ADJUST_X.get()),
                        VisionConstants.SHOOTER_CAM_POSE_Y + Units.inchesToMeters(VisionConstants.SHOOTER_CAM_ADJUST_Y.get()),
                        VisionConstants.SHOOTER_CAM_POSE_Z + Units.inchesToMeters(VisionConstants.SHOOTER_CAM_ADJUST_Z.get()),

                        new Rotation3d(
                                Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_ROLL + VisionConstants.SHOOTER_CAM_ADJUST_ROLL.get()),
                                Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_PITCH + VisionConstants.SHOOTER_CAM_ADJUST_PITCH.get()),
                                Units.degreesToRadians(VisionConstants.SHOOTER_CAM_POSE_YAW + VisionConstants.SHOOTER_CAM_ADJUST_YAW.get()))),

        };
    }

    public Vision(PhotonCamera[] cameras) {
        this.cameras = cameras;
    }

    @Override
    public void periodic() {

        Pose3d[] cameraPoses = getAdjustedCameraPoses();

        for (int cameraIndex = 0; cameraIndex < cameraPoses.length; cameraIndex++) {

            Pose3d cameraPoseEstimation;
            Pose2d robotPoseEstimation;

            List<Pose3d> tagPoses = new ArrayList<>();

            List<PhotonPipelineResult> result = cameras[cameraIndex].getAllUnreadResults();
            PhotonPipelineResult latestResult = result.get(result.size() - 1);

            double timestamp = latestResult.getTimestampSeconds();

            boolean rejectPose = false;

            if (!latestResult.hasTargets()) {
                continue;
            }

            cameraPoseEstimation = MathUtils.getPose3dFromTransform3d(latestResult.getMultiTagResult().get().estimatedPose.best);


            // Check the distance to each tag used in the multi-tag solution
            for (int id : latestResult.getMultiTagResult().get().fiducialIDsUsed) {

                // only relevant for custom apriltag layout
                if (fieldLayout.getTagPose(id).isEmpty()) {
                    continue;
                }


                Pose3d tagPose = fieldLayout.getTagPose(id).get();
                double distance = tagPose.getTranslation().getDistance(cameraPoseEstimation.getTranslation());


                if (distance < VisionConstants.MAX_ACCEPTABLE_TAG_RANGE) {
                    tagPoses.add(tagPose);
                } else {
                    rejectPose = true;
                }

            }

            if (tagPoses.isEmpty()) {
                rejectPose = true;
            }

            robotPoseEstimation =
                    cameraPoseEstimation.transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse()).toPose2d();

            Pose3d robotPoseEstimation3d = cameraPoseEstimation.transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse());


            if (robotPoseEstimation.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                    || robotPoseEstimation.getX() > FieldConstants.FIELD_LENGTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS
                    || robotPoseEstimation.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                    || robotPoseEstimation.getY() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS)

            {
                rejectPose = true;
            }

            if (robotPoseEstimation3d.getZ() > VisionConstants.MAX_Z_ERROR) {
                rejectPose = true;
            }

            if (latestResult.targets.get(0).getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                rejectPose = true;
            }

            if (rejectPose) {
                continue;
            }

            double totalDistance = 0.0;

            for (Pose3d tagPose : tagPoses) {
                totalDistance += tagPose.getTranslation().getDistance(cameraPoseEstimation.getTranslation());
            }

            double avgDistance = totalDistance / tagPoses.size();

            double xyStdDev = VisionConstants.BASE_VISION_XY_STD_DEV * Math.pow(avgDistance, 2.0) / tagPoses.size();
            double thetaStdDev = VisionConstants.BASE_VISION_THETA_STD_DEV * Math.pow(avgDistance, 2.0) / tagPoses.size();

            robotState.addVisionMeasurement(new VisionMeasurement(robotPoseEstimation, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));

        }

    }


}
