package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AgitateWithIntake;
import frc.robot.commands.AimAtHub3NoTurret;
import frc.robot.commands.AimAtHubWithChassis;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.FireShooterCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FlipOverYUtil;

import java.util.Set;

public class PreloadDumpRight extends SequentialCommandGroup {
    public PreloadDumpRight(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("PRELOAD DUMP RIGHT");
        addRequirements(swerveSubsystem, indexerSubsystem, turretSubsystem, shooterSubsystem, intakeSubsystem);
        addCommands(
                Commands.defer(() -> {
                    // Must use 'return' here because of the curly braces
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_RIGHT);
                                swerveSubsystem.resetOdometry(startPose);
                            }),
                            new SequentialCommandGroup(
                                    // Added AllianceFlipUtil.apply here as well so Red works!
                                    new DriveToPointContinuous(swerveSubsystem, FlipOverYUtil.apply(AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION)), 0.25)),
                                    new DriveToPoint(swerveSubsystem, FlipOverYUtil.apply(AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT)), 0.5),
                                    new ParallelRaceGroup(
                                            new AimAtHubWithChassis(swerveSubsystem, () -> 0.0, () -> 0.0),
                                            new AimAtHub3NoTurret(hoodSubsystem, shooterSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                            new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem),
                                            new AgitateWithIntake(intakeSubsystem)
                                    )
                    );
                }, Set.of(swerveSubsystem, shooterSubsystem, indexerSubsystem, turretSubsystem))
        );
    }
}