package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class TurretTestCommand extends Command {

    SwerveSubsystem swerveSubsystem;
    TurretSubsystem turretSubsystem;
    Pose2d targetPose;
    Pose2d currentPose;

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

        Pose2d currentPose = swerveSubsystem.getPose();
        Logger.recordOutput("Turret/TargetPose", targetPose);
        double targetAngle = targetPose.getTranslation().
                minus(currentPose.getTranslation())
                .getAngle()
                .minus(swerveSubsystem.getPose().getRotation())
                .getRadians();

        turretSubsystem.setTurret(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setVoltage(0.0);
    }

}
