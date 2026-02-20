package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class BallDetection extends SubsystemBase {

    private Pose3d ball;
    private SwerveSubsystem swerveSubsystem;
    private PhotonCamera camera;

    public BallDetection(PhotonCamera camera, SwerveSubsystem swerveSubsystem) {
        this.camera = camera;
        this.swerveSubsystem = swerveSubsystem;
    }

    private Pose3d getAdjustedCameraPose() {
        return new Pose3d(
                Units.inchesToMeters(VisionConstants.BALL_CAM_POSE_X) + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_X.get()),
                Units.inchesToMeters(VisionConstants.BALL_CAM_POSE_Y) + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_Y.get()),
                Units.inchesToMeters(VisionConstants.BALL_CAM_POSE_Z) + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_Z.get()),
                new Rotation3d(
                        Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_ROLL + VisionConstants.BALL_CAM_ADJUST_ROLL.get()),
                        Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_PITCH + VisionConstants.BALL_CAM_ADJUST_PITCH.get()),
                        Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_YAW + VisionConstants.BALL_CAM_ADJUST_YAW.get())
                )
        );
    }

    private Pose3d detectBalls() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        // Check if we have any results
        if (results == null || results.isEmpty()) {
            return null;
        }

        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        if (!latestResult.hasTargets()) {
            return null;
        }

        // For color detection, just get the best target (no class ID filtering)
        PhotonTrackedTarget ballTarget = latestResult.getBestTarget();

        if (ballTarget == null) {
            return null;
        }

        Pose3d cameraPose = getAdjustedCameraPose();
        double tx = Units.degreesToRadians(ballTarget.getYaw());
        double ty = Units.degreesToRadians(ballTarget.getPitch());

        Translation3d ballPos = new Translation3d(
                Math.cos(ty) * Math.cos(tx),
                Math.cos(ty) * Math.sin(tx),
                Math.sin(ty)
        );

        double t = (FieldConstants.BALL_HEIGHT_METERS - cameraPose.getZ()) / ballPos.getZ();
        Translation3d ballPoseCamera = MathUtils.getTranslation3dFromPose3d(cameraPose).plus(ballPos.times(t));
        Pose3d ballPoseRobot = new Pose3d(ballPoseCamera, Rotation3d.kZero)
                .transformBy(MathUtils.getTransform3dFromPose3d(cameraPose).inverse());

        return new Pose3d(swerveSubsystem.getPose())
                .transformBy(new Transform3d(ballPoseRobot.getTranslation(), Rotation3d.kZero));
    }

    public void updateBallPose() {
        ball = detectBalls();
    }

    @Override
    public void periodic() {
        updateBallPose();
        Logger.recordOutput("BallDetection/See_Balls", hasBall());
        if (hasBall()) {
            Logger.recordOutput("BallDetection/BallPose", getBallPose());
        }
    }

    /**
     * Check if a ball is currently detected
     * @return true if a ball is detected, false otherwise
     */
    public boolean hasBall() {
        return ball != null;
    }

    /**
     * Get the 2D pose of the detected ball
     * @return the ball's pose, or null if no ball is detected
     */
    public Pose2d getBallPose() {
        return ball != null ? ball.toPose2d() : null;
    }

}