package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTestCommand extends Command {

    SwerveSubsystem swerveSubsystem;
    TurretSubsystem turretSubsystem;
    Pose2d targetPose;

    public TurretTestCommand(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        turretSubsystem.resetPID();
    }

    @Override
    public void execute() {

        double targetAngle = targetPose.getTranslation().
                minus(swerveSubsystem.getPose().getTranslation())
                .getAngle()
                .minus(swerveSubsystem.getPose().getRotation())
                .getDegrees();

        turretSubsystem.setTurret(targetAngle);

    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setTurret(0);
    }

}
