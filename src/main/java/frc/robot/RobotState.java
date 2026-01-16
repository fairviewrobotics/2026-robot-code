package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Vision;
import frc.robot.utils.OdometryMeasurement;
import frc.robot.utils.VisionMeasurement;
import org.photonvision.PhotonCamera;

public class RobotState {

    private static RobotState instance;


    private final Vision vision = new Vision(
            new PhotonCamera[] {new PhotonCamera(NetworkTableInstance.getDefault(), "shooter_cam")}
    );

    public static RobotState getInstance() {
        if (instance == null) {
            instance = new RobotState();
        }
        return instance;
    }

    private SwerveDrivePoseEstimator poseEstimator;
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(Constants.BUFFER_SIZE_SECONDS);

    private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
            TimeInterpolatableBuffer.createBuffer(Constants.BUFFER_SIZE_SECONDS);

    public RobotState() {

        SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
                DriveConstants.KINEMATICS,
                new Rotation2d(),
                new SwerveModulePosition[]{},
                new Pose2d(),
                VecBuilder.fill(VisionConstants.ODOMETRY_XY_STD_DEV.get(), VisionConstants.ODOMETRY_XY_STD_DEV.get(), VisionConstants.ODOMETRY_THETA_STD_DEV.get()),
                VecBuilder.fill(VisionConstants.BASE_VISION_THETA_STD_DEV, VisionConstants.BASE_VISION_THETA_STD_DEV, VisionConstants.BASE_VISION_THETA_STD_DEV)
        );


    }

    public void addVisionMeasurement(VisionMeasurement visionMeasurement) {
        poseEstimator.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(), visionMeasurement.stdDevs());
    }

    public void addOdometryMeasurement(OdometryMeasurement odometryMeasurement) {
        Pose2d odometryPose = poseEstimator.update(odometryMeasurement.gyroAngle(), odometryMeasurement.modulePositions());
        poseEstimator.updateWithTime(odometryMeasurement.timestamp(), odometryMeasurement.gyroAngle(), odometryMeasurement.modulePositions());
        poseBuffer.addSample(odometryMeasurement.timestamp(), odometryPose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

}
