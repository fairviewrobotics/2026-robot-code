package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    private SlewRateLimiter magnitudeSlew = new SlewRateLimiter(0);
    private SlewRateLimiter thetaSlew = new SlewRateLimiter(20);
    static final Lock odometryLock = new ReentrantLock();

    private Rotation2d gyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] previousModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    public SwerveDrivePoseEstimator poseEstimator;

    public SwerveSubsystem() {
        this.resetGyro();
        this.modules =
                new SwerveModule[] {
                        new SwerveModule(DriveConstants.FRONT_LEFT_CONFIG),
                        new SwerveModule(DriveConstants.FRONT_RIGHT_CONFIG),
                        new SwerveModule(DriveConstants.REAR_LEFT_CONFIG),
                        new SwerveModule(DriveConstants.REAR_RIGHT_CONFIG),
                };

        this.poseEstimator =
            new SwerveDrivePoseEstimator(
                    DriveConstants.KINEMATICS,
                    gyroRotation,
                    previousModulePositions,
                    Pose2d.kZero
            );
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, this.gyro.getRotation2d())
                : new ChassisSpeeds(vx, vy, vr);

        SwerveModuleState[] states = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_XY_SPEED_MPS.get());

        for (int i = 0; i < modules.length; i++) {
            modules[i].setVelocity(states[i]);
        }

        Logger.recordOutput("swerve/target angle", states[0].angle.getRadians());
        Logger.recordOutput("swerve/CommandedSpeeds", speeds);
    }

    public void periodic() {
        odometryLock.lock();
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isDisabled()) {
            for (var module : modules) { module.stop(); }
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps();

        double[] absolutePositions = new double[4];

        gyroRotation = gyro.getRotation2d();

        int sampleCount = Integer.MAX_VALUE;
        for (var module : modules) {sampleCount = Math.min(sampleCount, module.getOdometryTimestamps().length);}
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPosition(i);
                if (i == sampleCount - 1) {
                    absolutePositions[moduleIndex] = modulePositions[moduleIndex].angle.getDegrees();
                }
            }
            

            poseEstimator.updateWithTime(sampleTimestamps[i], gyroRotation, modulePositions);
        }

        double[] swerveStateArray = new double[8];
        var states = getModuleStates();
        for (int i = 0; i < 4; i++) {
            swerveStateArray[i * 2] = states[i].angle.getRadians();
            swerveStateArray[i * 2 + 1] = states[i].speedMetersPerSecond;
        }

        Logger.recordOutput("swerve/advantagescope swerve states", swerveStateArray);
        Logger.recordOutput("swerve/absolute encoder degrees", absolutePositions);
        Logger.recordOutput("swerve/module states", getModuleStates());
        Logger.recordOutput("swerve/gyro", gyroRotation);
        Logger.recordOutput("odometry/robot pose", poseEstimator.getEstimatedPosition());

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule m : this.modules) {
            states[m.moduleNumber] = m.getState();
        }
        return states;
    }


    public ChassisSpeeds getRobotVelocity() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
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
