package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class SwerveSubsystem extends SubsystemBase {

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private SwerveModule[] modules;
    private SlewRateLimiter magnitudeSlew;
    private SlewRateLimiter thetaSlew;
    static final Lock odometryLock = new ReentrantLock();


    private Rotation2d gyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] previousModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };


    public SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.KINEMATICS,
                    gyroRotation,
                    previousModulePositions,
                    Pose2d.kZero
            );

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules =
                new SwerveModule[] {
                        new SwerveModule(DriveConstants.FRONT_LEFT_CONFIG),
                        new SwerveModule(DriveConstants.FRONT_RIGHT_CONFIG),
                        new SwerveModule(DriveConstants.REAR_LEFT_CONFIG),
                        new SwerveModule(DriveConstants.REAR_RIGHT_CONFIG),
                };
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean slewRates) {

        double vx = slewRates ? magnitudeSlew.calculate(translation.getX()) : translation.getX();
        double vy = slewRates ? magnitudeSlew.calculate(translation.getY()) : translation.getY();
        double vr = slewRates ? thetaSlew.calculate(rotation) : rotation;

        ChassisSpeeds speeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, this.getPose().getRotation())
                : new ChassisSpeeds(vx, vy, vr);

        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_XY_SPEED_MPS.get());

        for (int i = 0; i < modules.length; i++) {
            modules[i].setVelocity(states[i]);
        }

        // Logging
        Logger.recordOutput("swerve/CommandedSpeeds", speeds);
    }

    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getModulePosition();
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - previousModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                previousModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            gyroRotation = new Rotation2d(gyro.getAngle());

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], gyroRotation, modulePositions);

        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule m : this.modules) {
            states[m.moduleNumber] = m.getState();
        }
        return states;
    }

    public ChassisSpeeds getRobotVelocity() {
        return DriveConstants.KINEMATICS.toChassisSpeeds();
    }

    public ChassisSpeeds getFieldVelocity() {
        ChassisSpeeds robotRelative = this.getRobotVelocity();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, gyroRotation);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

}
