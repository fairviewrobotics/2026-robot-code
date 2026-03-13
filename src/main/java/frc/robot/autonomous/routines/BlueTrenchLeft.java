package frc.robot.autonomous.routines;

import com.fasterxml.jackson.databind.ext.SqlBlobSerializer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * Left relative to driver
 */

public class BlueTrenchLeft extends SequentialCommandGroup {
    public BlueTrenchLeft(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("BLUE TRENCH LEFT");
        addRequirements(swerveSubsystem);
        addCommands(
                new SequentialCommandGroup(

                        new InstantCommand(() -> {
                        Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                        swerveSubsystem.resetOdometry(startPose);
                        }),
                        
                        new ParallelDeadlineGroup(
                                new SequentialCommandGroup(
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS), 0.75),
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5),
                                        new ParallelDeadlineGroup(
                                                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 1.0),
                                                new IntakeCommand(intakeSubsystem)
                                        ),
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5),
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.75),
                                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0),
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
                                new ShooterCommand(shooterSubsystem, 1000, 1000)
                        )
                
                )
                // new SequentialCommandGroup(
                //         new InstantCommand(()->swerveSubsystem.resetOdometry(AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT))),
                //         new ParallelCommandGroup(
                //                 new SequentialCommandGroup(),
                //                 new ShooterCommand(shooterSubsystem, 0, 0)
                //         )
                // )
        );
    }
}
