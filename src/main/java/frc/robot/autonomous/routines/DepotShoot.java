package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class DepotShoot extends SequentialCommandGroup {

    public DepotShoot(SwerveSubsystem swerveSubsystem, SwerveDrive swerveDrive) {
        setName("DepotShoot");
        addRequirements(swerveSubsystem);
        addCommands(
                new DriveToPoint(swerveSubsystem, swerveDrive.getPose(), FieldConstants.DEPOT_INTAKE_POSE, 0.7)
                        .andThen(new DriveToPoint(swerveSubsystem, swerveDrive.getPose(), FieldConstants.AUTO_SHOOT_POSE, 0.7))
        );
    }

}
