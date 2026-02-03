package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TurretTestCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class DepotShoot extends SequentialCommandGroup {

    public DepotShoot(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem) {
        setName("DEPOT_SHOOT");
        addRequirements(swerveSubsystem);
        addCommands(
                new DriveToPoint(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.DEPOT_INTAKE_POSE, 0.25),
                new TurretTestCommand(swerveSubsystem, turretSubsystem, FieldConstants.AUTO_SHOOT_POSE),
                new DriveToPoint(swerveSubsystem, swerveSubsystem.getPose(), FieldConstants.AUTO_SHOOT_POSE, 0.25)
        );
    }

}