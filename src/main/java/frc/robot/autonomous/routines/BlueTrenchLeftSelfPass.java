package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Set;

public class BlueTrenchLeftSelfPass extends SequentialCommandGroup {


    public BlueTrenchLeftSelfPass(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("BLUE TRENCH LEFT SELF PASS");
        addRequirements(swerveSubsystem);
        addCommands(Commands.defer(() -> {
            double AUTO_SCALAR = Preferences.getDouble("Auto/AUTO_SCALAR", 0.2);

            return new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                        swerveSubsystem.resetOdometry(startPose);
                    }),

                    new ZeroTurretCommand(turretSubsystem).withTimeout(0.5),

                    new SequentialCommandGroup(
                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_CONTINUOUS), 1.0 * AUTO_SCALAR),
                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 1.0 * AUTO_SCALAR),
                            new ParallelDeadlineGroup(
                                    new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 1.0 * AUTO_SCALAR),
                                    new IntakeCommand(intakeSubsystem)
                            ),
                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_DROPOFF), 1.0 * AUTO_SCALAR),
                            new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0 * AUTO_SCALAR),
                            new ParallelDeadlineGroup(
                                    new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem).withTimeout(5.0),
                                    new AimAtHub3(hoodSubsystem, shooterSubsystem, turretSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                    new AgitateWithIntake(intakeSubsystem)
                            ),

                            new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP_2), 1.0 * AUTO_SCALAR),
                            new ParallelDeadlineGroup(
                                    new SequentialCommandGroup(
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_ARC_DROPOFF), 0.2 * AUTO_SCALAR),
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_ARC_END_DROPOFF), 0.2 * AUTO_SCALAR),
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_SELF_PASS_END_POINT), 0.2 * AUTO_SCALAR)

                                    ),
                                    new IntakeCommand(intakeSubsystem),
                                    new AimAtHub3(hoodSubsystem, shooterSubsystem, turretSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_PASS_LEFT_POSE.getTranslation()),
                                    new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem)
                            )


                    )

            );

        }, Set.of(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem))

        );

    }

}
