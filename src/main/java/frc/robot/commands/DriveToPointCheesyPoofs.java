package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.MathUtils;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;


public class DriveToPointCheesyPoofs extends Command {

    private ProfiledPIDController driveController;
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    Constants.DrivebaseConstants.AUTO_ROTATION_P.get(),
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            Constants.MAX_ANGULAR_SPEED,
                            Constants.MAX_ANGULAR_SPEED/2),
                    0.02);
    private SwerveSubsystem driveSubsystem;
    private Pose2d currentPose;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private Pose2d targetLocation;

    public DriveToPointCheesyPoofs(
            SwerveSubsystem driveSubsystem,
            Pose2d currentPose,
            Pose2d targetLocation,
            double constraintFactor) {
        this.driveSubsystem = driveSubsystem;
        this.targetLocation = targetLocation;
        this.currentPose = currentPose;
        this.driveController =
                new ProfiledPIDController(
                        Constants.DrivebaseConstants.DECELERATION_P.get(),
                        0.0,
                        0.0,
                        new TrapezoidProfile.Constraints(
                                Constants.MAX_SPEED * constraintFactor,
                                Constants.MAX_SPEED/2
                                        * constraintFactor),
                        0.02);
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // arm center is the same as the robot center when stowed, so can use field to
        // robot
        Pose2d currentPose = driveSubsystem.getPose();

        driveController.reset(
                currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(
                                driveSubsystem.getFieldVelocity().vxMetersPerSecond,
                                driveSubsystem.getFieldVelocity().vyMetersPerSecond)
                                .rotateBy(
                                        targetLocation
                                                .getTranslation()
                                                .minus(
                                                        driveSubsystem
                                                                .getPose()
                                                                .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(
                currentPose.getRotation().getRadians(),
                driveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(2.0));

        driveController.setTolerance(0.01);
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        Logger.recordOutput("DriveToPose/currentPose", currentPose);
        Logger.recordOutput("DriveToPose/targetLocation", targetLocation.toString());
        Logger.recordOutput("DriveToPose/targetPose", targetLocation);

        driveController.setP(Preferences.getDouble("DECELERATION_P", 1.5));
        driveController.setD(Preferences.getDouble("DECELERATION_D", 0.0));

        thetaController.setP(Preferences.getDouble("AUTO_ROTATION_P", 0.0));
        thetaController.setD(Preferences.getDouble("AUTO_ROTATION_D", 0.0));

        double currentDistance =
                currentPose.getTranslation().getDistance(targetLocation.getTranslation());
        double ffScaler =
                MathUtil.clamp(
                        (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
        driveErrorAbs = currentDistance;
        Logger.recordOutput("DriveToPose/ffScaler", ffScaler);
        double driveVelocityScalar =
                driveController.getSetpoint().velocity * ffScaler
                        + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

        // Calculate theta speed
        double thetaVelocity =
                thetaController.getSetpoint().velocity * ffScaler
                        + thetaController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetLocation.getRotation().getRadians());
        thetaErrorAbs =
                Math.abs(
                        currentPose.getRotation().minus(targetLocation.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        var translationMag = currentPose.getTranslation().minus(targetLocation.getTranslation());

        // Command speeds
        var driveVelocity =
                MathUtils.getPoseFromRotation(
                                currentPose
                                        .getTranslation()
                                        .minus(targetLocation.getTranslation())
                                        .getAngle())
                        .transformBy(
                                MathUtils.getTransform2dFromTranslation(
                                        new Translation2d(driveVelocityScalar, 0.0)))
                        .getTranslation();
        double xVel = translationMag.getNorm() * Math.cos(translationMag.getAngle().getRadians());
        double yVel = translationMag.getNorm() * Math.sin(translationMag.getAngle().getRadians());
        Translation2d driveVals = new Translation2d(driveVelocity.getX(), driveVelocity.getY());
        driveSubsystem.drive(driveVals, thetaVelocity, true);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(Translation2d.kZero, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        return targetLocation.equals(null)
                || (driveController.atGoal() && thetaController.atGoal());
    }

}
