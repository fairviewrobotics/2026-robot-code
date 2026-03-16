package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;

import java.util.Set;

public class BlueTrenchLeftWithPreload extends SequentialCommandGroup {
    public BlueTrenchLeftWithPreload(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("BLUE TRENCH LEFT WITH PRELOAD");
        addRequirements(swerveSubsystem, intakeSubsystem, turretSubsystem);
        addCommands(
            Commands.defer(() -> {
                double AUTO_SCALAR = Preferences.getDouble("Auto/AUTO_SCALAR", 0.2);
                return new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                            swerveSubsystem.resetOdometry(startPose);
                        }),
                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION), 0.5 * AUTO_SCALAR),
                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0 * AUTO_SCALAR),
                        new ParallelRaceGroup(
                                // Do not flip this one, it does it in the command
                                new AimAtHub3NoTurret(hoodSubsystem, shooterSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                new AimAtHubWithChassis(swerveSubsystem, () -> 0.0 , () -> 0.0),
                                new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem).withTimeout(3.0),
                                new AgitateWithIntake(intakeSubsystem)
                        ),
                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION), 1.0 * AUTO_SCALAR),
                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.5 * AUTO_SCALAR),
                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.5 * AUTO_SCALAR),
                        new ParallelDeadlineGroup(
                                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 1.0 * AUTO_SCALAR),
                                new IntakeCommand(intakeSubsystem)
                        ),
                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_DROPOFF), 1.0 * AUTO_SCALAR),
                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.5 * AUTO_SCALAR),
                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION), 0.5 * AUTO_SCALAR),
                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0 * AUTO_SCALAR),
                        new ParallelDeadlineGroup(
                                new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem).withTimeout(4.0),
                                // Do not flip this one, it does it in the command
                                new AimAtHub3NoTurret(hoodSubsystem, shooterSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                new AimAtHubWithChassis(swerveSubsystem, () -> 0.0, () -> 0.0),
                                new AgitateWithIntake(intakeSubsystem)
                        )
                );
                }, Set.of(swerveSubsystem, intakeSubsystem, shooterSubsystem, indexerSubsystem))
        );
    }
}
