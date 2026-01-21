package frc.robot.utils;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Camera {

    private PhotonCamera photonCamera;

    private final String name;
    private final Transform3d offset;
    private NetworkTable table;
    private boolean enabled;

    public Camera(Boolean enabled, String name, Transform3d offset) {
        this.enabled = enabled;
        this.name = name;
        this.offset = offset;
        this.photonCamera = new PhotonCamera(name);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public Boolean isEnabled() {
        return enabled;
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }



    public PhotonPipelineResult getLatestResult() {
        return photonCamera.getAllUnreadResults().get(0);
    }

    public PhotonTrackedTarget getBestTarget() {
        if (!getLatestResult().hasTargets()) return null;
        return getLatestResult().getBestTarget();
    }

    public double getTX() {
        PhotonTrackedTarget target = getBestTarget();
//
//        if (!(target == null) && ((RobotController.getFPGATime() - horizontalTime) < 300)) {
//            return oldTX - (lastKnowHeading - SwerveSubsystem.getGyro();
//        }
        return (target == null) ? 0.0 : target.getYaw();
    }

    public double getTY() {
        PhotonTrackedTarget target = getBestTarget();
        return (target == null) ? 0.0 : target.getPitch();
    }

    public double getTagArea() {
        PhotonTrackedTarget target = getBestTarget();
        return (target == null) ? 0.0 : target.getArea();
    }

    public int getFiducialId() {
        PhotonTrackedTarget target = getBestTarget();
        return (target == null) ? -1 : target.getFiducialId();
    }

    double getSkew() {
        double skew = getTX();
        if (skew > -45) {
            return skew + 90;
        }
        return skew;
    }

    public double getDistance() {
        PhotonTrackedTarget target = getBestTarget();
        double cameraHeight = offset.getZ();
        double distance = (5 - cameraHeight) /
                Math.tan(offset.getRotation().getQuaternion().getY() + getTY());
        return distance;
    }



}
