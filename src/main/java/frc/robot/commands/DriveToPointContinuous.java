package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drive to point but don't stop
 * For auto paths
 */

public class DriveToPointContinuous extends Command {
    private final SwerveSubsystem swerveSubsystem; // Added final for safety
    private final ProfiledPIDController thetaController;
    private final Pose2d targetPose;
    private final double constraintFactor;

    // Added swerveSubsystem to constructor
    public DriveToPointContinuous(SwerveSubsystem swerveSubsystem, Pose2d targetPose, double constraintFactor) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetPose = targetPose;
        this.constraintFactor = constraintFactor;

        this.thetaController = new ProfiledPIDController(
                Preferences.getDouble("DriveToPoint/AutoRotationP", Constants.DrivebaseConstants.AUTO_ROTATION_P),
                0.0, 0.0,
                new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED/2, Constants.MAX_ANGULAR_SPEED)
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(Units.degreesToRadians(2.0));
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Capture the target location at initialization time
        Pose2d currentPose = swerveSubsystem.getPose();

        // Update PID values from preferences
        thetaController.setP(Preferences.getDouble("DriveToPoint/AutoRotationP", Constants.DrivebaseConstants.AUTO_ROTATION_P));

        thetaController.reset(
                currentPose.getRotation().getRadians(),
                swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(5.0));

    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();

        double deltaX = targetPose.getX() - currentPose.getX();
        double deltaY = targetPose.getY() - currentPose.getY();
        double angleToTarget = Math.atan2(deltaY, deltaX);

        double xVelocity = Constants.MAX_SPEED * constraintFactor * Math.cos(angleToTarget);
        double yVelocity = Constants.MAX_SPEED * constraintFactor * Math.sin(angleToTarget);

        double omegaVelocity = thetaController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        swerveSubsystem.drive(new Translation2d(xVelocity, yVelocity), omegaVelocity, true);
    }

    @Override
    public boolean isFinished() {
        return swerveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.2;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
