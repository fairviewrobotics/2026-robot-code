package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Set;

public class PreloadDumpMiddle extends SequentialCommandGroup {

    public PreloadDumpMiddle(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("PRELOAD DUMP MIDDLE");
        addRequirements(swerveSubsystem, indexerSubsystem, turretSubsystem, shooterSubsystem, intakeSubsystem);
        addCommands(
            Commands.defer(() -> {
                return new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                        swerveSubsystem.resetOdometry(startPose);
                    }),
                    new SequentialCommandGroup(

                    )
                );
            }, Set.of(swerveSubsystem, indexerSubsystem, shooterSubsystem))
        );
    }

}
