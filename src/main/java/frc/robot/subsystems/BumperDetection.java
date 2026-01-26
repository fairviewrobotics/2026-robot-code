package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public class BumperDetection extends SubsystemBase {

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

    public BumperDetection(PhotonCamera camera) {
        this.camera = camera;
    }

    public void setPipeline(int pipeline) {
        camera.setPipelineIndex(pipeline);
    }

    private boolean seeBumpers() {

        Pose3d cameraPose = getAdjustedCameraPose();

        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        PhotonPipelineResult latestResult = result.get(result.size() - 1);

        if (!latestResult.hasTargets()) {
            return false;
        }

        Optional<PhotonTrackedTarget> bumpers = latestResult.getTargets().stream()
                .filter(target -> target.getDetectedObjectClassID() == 1).findFirst();

        if (bumpers.isEmpty()) {
            return false;
        }
        double tx = bumpers.get().yaw;
        double ty = bumpers.get().pitch;

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

            if (allianceOpt.isEmpty()) {
                return false;
            }


            DriverStation.Alliance alliance = allianceOpt.get();
            int pipeline = (alliance == DriverStation.Alliance.Red)
                    ? 0
                    : 1;

            this.setPipeline(pipeline);

            return ty >= VisionConstants.BUMPER_DETECTION_RETRACT_TY_DEGREES.get();

    }

}
