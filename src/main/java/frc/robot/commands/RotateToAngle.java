package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateToAngle extends Command {

    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    Constants.DrivebaseConstants.AUTO_ROTATION_P.get(),
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            Constants.MAX_ANGULAR_SPEED,
                            Constants.MAX_ANGULAR_SPEED/2),
                    0.02);

    private SwerveSubsystem swerveSubsystem;
    private Vision vision;
    private Rotation2d currentAngle;
    private double thetaErrorAbs;
    private Rotation2d targetAngle;


    public RotateToAngle(SwerveSubsystem swerveSubsystem, Rotation2d targetAngle, Rotation2d currentAngle) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetAngle = targetAngle;
        this.currentAngle = currentAngle;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        thetaController.reset(
                currentAngle.getRadians(),
                swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond);
        thetaController.setTolerance(Units.degreesToRadians(2.0));
    }

    @Override
    public void execute() {

        double thetaVelocity =
                thetaController.getSetpoint().velocity
                        + thetaController.calculate(
                        currentAngle.getRadians(),
                        targetAngle.getRadians());
        thetaErrorAbs =
                Math.abs(
                        currentAngle.minus(targetAngle).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

        swerveSubsystem.drive(Translation2d.kZero, thetaVelocity, true, true);

    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(Translation2d.kZero, 0.0, false, true);
    }

    @Override
    public boolean isFinished() {
        return thetaController.atGoal();
    }

}
