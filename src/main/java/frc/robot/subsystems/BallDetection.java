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

    // better name for this?
    private Pose3d ball;
    private SwerveSubsystem swerveSubsystem;
    PhotonCamera camera;

    private Pose3d getAdjustedCameraPose() {
                return new Pose3d(

                        VisionConstants.BALL_CAM_POSE_X + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_X.get()),
                        VisionConstants.BALL_CAM_POSE_Y + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_Y.get()),
                        VisionConstants.BALL_CAM_POSE_Z + Units.inchesToMeters(VisionConstants.BALL_CAM_ADJUST_Z.get()),

                        new Rotation3d(
                                Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_ROLL + VisionConstants.BALL_CAM_ADJUST_ROLL.get()),
                                Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_PITCH + VisionConstants.BALL_CAM_ADJUST_PITCH.get()),
                                Units.degreesToRadians(VisionConstants.BALL_CAM_POSE_YAW + VisionConstants.BALL_CAM_ADJUST_YAW.get())));

    }

    public BallDetection(PhotonCamera camera) {
        this.camera = camera;
    }

    private Pose3d detectBalls() {

        Pose3d cameraPose = getAdjustedCameraPose();

        List<PhotonPipelineResult> result = camera.getAllUnreadResults();

        if (result.isEmpty()) {
            return null;
        }

        PhotonPipelineResult latestResult = result.get(result.size() - 1);

        if (!latestResult.hasTargets()) {
            return null;
        }

        Optional<PhotonTrackedTarget> balls = latestResult.getTargets().stream()
                .filter(target -> target.getDetectedObjectClassID() == 1).findFirst();

        if (balls.isEmpty()) {
            return null;
        }

        double tx = balls.get().yaw;
        double ty = balls.get().pitch;

        Translation3d ballPos = new Translation3d(Math.cos(ty)*Math.cos(tx),
                Math.cos(ty)*Math.sin(tx),
                Math.sin(ty));

        double t = (FieldConstants.BALL_HEIGHT_METERS - cameraPose.getZ()) / ballPos.getZ();
        Translation3d ballPoseCamera = MathUtils.getTranslation3dFromPose3d(cameraPose).plus(ballPos.times(t));
        Pose3d ballPoseRobot = new Pose3d(ballPoseCamera, Rotation3d.kZero).transformBy(MathUtils.getTransform3dFromPose3d(cameraPose).inverse());
        return ball = new Pose3d(swerveSubsystem.getPose()).transformBy(new Transform3d(ballPoseRobot.getTranslation(), Rotation3d.kZero));
    }

    public void updateBallPose() {

        ball = detectBalls();

    }

    @Override
    public void periodic() {
        updateBallPose();
        Logger.recordOutput("BallDetection/Ball Pose", getBallPose());
    }

    public Pose2d getBallPose() {
        if (ball == null) {
            return null;
        }
        return ball.toPose2d();
    }

}
