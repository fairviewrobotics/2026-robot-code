package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants; // Assume offset is stored here
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTestCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final Pose2d targetPose;

    // The turret's physical location relative to the robot center (in meters)
    // Positive X is forward, Positive Y is left.
    private final Translation2d turretOffset = ShootingConstants.TURRET_OFFSET;

    public TurretTestCommand(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.targetPose = targetPose;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        turretSubsystem.resetPID();
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerveSubsystem.getPose();

        Translation2d turretFieldLocation = robotPose.getTranslation().plus(
                turretOffset.rotateBy(robotPose.getRotation())
        );

        Pose2d turretPose = new Pose2d(turretFieldLocation, robotPose.getRotation());

        double targetAngle = targetPose
                .minus(turretPose).getRotation().getRadians();

        turretSubsystem.setTurret(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setTurret(turretSubsystem.getTurretAngle());
    }
}