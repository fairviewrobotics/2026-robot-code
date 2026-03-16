package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;
import java.util.Set; // Needed for the requirements set in defer

public class PreloadDump extends SequentialCommandGroup {
    public PreloadDump(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("PRELOAD DUMP");
        addRequirements(swerveSubsystem, indexerSubsystem, turretSubsystem, shooterSubsystem, intakeSubsystem);
        addCommands(
                Commands.defer(() -> {
                    // Must use 'return' here because of the curly braces
                    return new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT);
                                swerveSubsystem.resetOdometry(startPose);
                            }),
                                new SequentialCommandGroup(
                                        new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TO_SHOOT_TRANSITION), 0.25),
                                        new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 0.5),
                                        new ParallelRaceGroup(
                                                // Do not flip this one, it does it in the command
                                                new AimAtHub3NoTurret(hoodSubsystem, shooterSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                                new AimAtHubWithChassis(swerveSubsystem, () -> 0.0 , () -> 0.0),
                                                new FireShooterCommand(shooterSubsystem, indexerSubsystem, turretSubsystem),
                                                new AgitateWithIntake(intakeSubsystem)
                                        )
                                )
                    );
                }, Set.of(swerveSubsystem, shooterSubsystem, indexerSubsystem, turretSubsystem))
        );
    }
}