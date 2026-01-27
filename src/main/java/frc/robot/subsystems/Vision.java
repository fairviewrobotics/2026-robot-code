package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {

    private static Vision instance;
    private final PhotonCamera[] cameras;
    private final SwerveSubsystem swerveSubsystem;
    private final AprilTagFieldLayout fieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    private final Pose3d[] baseCameraPoses = new Pose3d[] {
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

    Pose3d[] cameraPoses = getAdjustedCameraPoses();

    private Vision(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.cameras = new PhotonCamera[] {
                new PhotonCamera("rev tag cam")
        };
    }

    public static void init(SwerveSubsystem swerveSubsystem) {
        if (instance == null) {
            instance = new Vision(swerveSubsystem);
        }
    }

    public static Vision getInstance() {
        if (instance == null) {
            throw new IllegalStateException("Vision must be initialized with init() first");
        }
        return instance;
    }

    @Override
    public void periodic() {
        updatePose();
    }



    public void updatePose() {

        frc.robot.utils.TunableNumber.ifChanged(
                hashCode(),
                this::getAdjustedCameraPoses,
                VisionConstants.SHOOTER_CAM_ADJUST_X,
                VisionConstants.SHOOTER_CAM_ADJUST_Y,
                VisionConstants.SHOOTER_CAM_ADJUST_Z,
                VisionConstants.SHOOTER_CAM_ADJUST_ROLL,
                VisionConstants.SHOOTER_CAM_ADJUST_PITCH,
                VisionConstants.SHOOTER_CAM_ADJUST_YAW
        );


        for (int cameraIndex = 0; cameraIndex < cameraPoses.length; cameraIndex++) {

            List<PhotonPipelineResult> results = cameras[cameraIndex].getAllUnreadResults();

            if (results.isEmpty()) {
                continue;
            }

            PhotonPipelineResult latestResult = results.get(results.size() - 1);

            // Check if result has targets
            if (!latestResult.hasTargets()) {
                continue;
            }

            double timestamp = latestResult.getTimestampSeconds();

            boolean useMultitag = latestResult.multitagResult.isPresent();

            if (useMultitag) {

                Pose3d cameraPoseEstimation = MathUtils.getPose3dFromTransform3d(
                        latestResult.getMultiTagResult().get().estimatedPose.best);


                List<Pose3d> tagPoses = new ArrayList<>();

                for (int id : latestResult.getMultiTagResult().get().fiducialIDsUsed) {

                    // remove bad tags from the map at some point
                    if (fieldLayout.getTagPose(id).isEmpty()) {
                        continue;
                    }

                    Pose3d tagPose = fieldLayout.getTagPose(id).get();
                    double distance = tagPose.getTranslation().getDistance(cameraPoseEstimation.getTranslation());

                    if (distance < VisionConstants.MAX_ACCEPTABLE_TAG_RANGE) {
                        tagPoses.add(tagPose);
                    } else {
                        return;
                    }
                }

                if (tagPoses.isEmpty()) {
                    return;
                }

                Pose2d robotPoseEstimation = cameraPoseEstimation
                        .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse())
                        .toPose2d();

                Pose3d robotPoseEstimation3d = cameraPoseEstimation
                        .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse());

                if (robotPoseEstimation.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                        || robotPoseEstimation.getX() > FieldConstants.FIELD_LENGTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS
                        || robotPoseEstimation.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                        || robotPoseEstimation.getY() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS) {
                    return;
                }

                if (robotPoseEstimation3d.getZ() > VisionConstants.MAX_Z_ERROR) {
                    return;
                }

                if (!latestResult.targets.isEmpty()
                        && latestResult.targets.get(0).getPoseAmbiguity() > VisionConstants.MAX_POSE_AMBIGUITY) {
                    return;
                }

                // Calculate average distance to tags
                double totalDistance = 0.0;
                for (Pose3d tagPose : tagPoses) {
                    totalDistance += tagPose.getTranslation().getDistance(cameraPoseEstimation.getTranslation());
                }
                double avgDistance = totalDistance / tagPoses.size();

                // Calculate dynamic standard deviations
                double xyStdDev = (VisionConstants.BASE_VISION_XY_STD_DEV.get() * avgDistance / tagPoses.size());
                double thetaStdDev = VisionConstants.BASE_VISION_THETA_STD_DEV.get() * avgDistance / tagPoses.size();

                double baseXY = 0.005;

                swerveSubsystem.poseEstimator.addVisionMeasurement(
                        robotPoseEstimation,
                        timestamp,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            } else {

                PhotonTrackedTarget target = latestResult.targets.get(0);

                Pose3d singleTagPose = fieldLayout.getTagPose(target.getFiducialId()).get();
                Pose3d cameraPose = singleTagPose.transformBy(target.getBestCameraToTarget().inverse());

                Pose3d robotPoseEstimation =
                        cameraPose
                                .transformBy(MathUtils.getTransform3dFromPose3d(cameraPoses[cameraIndex]).inverse());

                double distance = singleTagPose.getTranslation().getDistance(cameraPose.getTranslation());

                boolean rejectPose =
                        VisionConstants.MAX_ACCEPTABLE_TAG_RANGE < distance ||
                                robotPoseEstimation.getX() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                                || robotPoseEstimation.getX() > FieldConstants.FIELD_LENGTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS
                                || robotPoseEstimation.getY() < -FieldConstants.FIELD_BORDER_MARGIN_METERS
                                || robotPoseEstimation.getY() > FieldConstants.FIELD_WIDTH_METERS + FieldConstants.FIELD_BORDER_MARGIN_METERS ||
                                robotPoseEstimation.getZ() > VisionConstants.MAX_Z_ERROR;

                if (rejectPose) {
                    return;
                }

                double xyStdDev = VisionConstants.SINGLE_TAG_DISTRUST_COEFFICIENT.get() * VisionConstants.BASE_VISION_XY_STD_DEV.get() * Math.pow(distance, 2.0);
                double thetaStdDev = VisionConstants.SINGLE_TAG_DISTRUST_COEFFICIENT.get() * VisionConstants.BASE_VISION_THETA_STD_DEV.get() * Math.pow(distance, 2.0);

                double baseXY = 0.005;

                swerveSubsystem.poseEstimator.addVisionMeasurement(
                        robotPoseEstimation.toPose2d(),
                        timestamp,
                        VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

            }

        }

    }

    /**
     * Get the current estimated robot pose from YAGSL.
     */

    public Pose2d getRobotPose() {
        return swerveSubsystem.poseEstimator.getEstimatedPosition();
    }

}