package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveLocalizer;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAtHub extends Command {

    private Pose2d currentPose;
    private Pose2d targetPose;
    private SwerveSubsystem swerveSubsystem;
    private SwerveLocalizer swerveLocalizer;

    public AimAtHub(SwerveSubsystem swerveSubsystem, SwerveLocalizer swerveLocalizer, Pose2d currentPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.swerveLocalizer = swerveLocalizer;
        this.currentPose = currentPose;
    }

    @Override
    public void initialize() {

    }



}
