package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;
import java.util.Set;

/**
 * Left relative to driver
 */
public class BlueTrenchLeft extends SequentialCommandGroup {
    public BlueTrenchLeft(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        double AUTO_SCALAR = Preferences.getDouble("Auto/AUTO_SCALAR", 0.2);
        setName("BLUE TRENCH LEFT");
        addRequirements(swerveSubsystem);

        addCommands(
                Commands.defer(() -> {
                    // The return statement is required because of the curly braces
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                                swerveSubsystem.resetOdometry(startPose);
                            }),

                            new ParallelDeadlineGroup(
                                    new SequentialCommandGroup(
                                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS), 0.75 * AUTO_SCALAR),
                                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5 * AUTO_SCALAR),
                                            new ParallelDeadlineGroup(
                                                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 1.0 * AUTO_SCALAR),
                                                    new IntakeCommand(intakeSubsystem)
                                            ),
                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5 * AUTO_SCALAR),
                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.75 * AUTO_SCALAR),
                                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0 * AUTO_SCALAR),
                                            new IndexerCommand(indexerSubsystem).withTimeout(4.0),

                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS), 0.75),
                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5),
                                            new ParallelDeadlineGroup(
                                                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 1.0),
                                                    new IntakeCommand(intakeSubsystem)
                                            ),
                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5),
                                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.75),
                                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0),
                                            new IndexerCommand(indexerSubsystem).withTimeout(4.0)
                                    ),
                                    new ShooterCommand(shooterSubsystem, turretSubsystem, 1000, 1000)
                            )
                    );
                }, Set.of(swerveSubsystem, intakeSubsystem, shooterSubsystem, indexerSubsystem))
        );
    }
}