package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.AgitateWithIntake;
import frc.robot.commands.AimAtHub3;
import frc.robot.commands.AimAtHub3NoTurret;
import frc.robot.commands.AimAtHubWithChassis;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.commands.FireShooterCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ZeroTurretCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.*;
import frc.robot.utils.AllianceFlipUtil;
import java.util.Set;

/**
 * Left relative to driver
 */
public class BlueTrenchLeft extends SequentialCommandGroup {
    public BlueTrenchLeft(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, IndexerSubsystem indexerSubsystem) {
        setName("BLUE TRENCH LEFT");
        addRequirements(swerveSubsystem);

        addCommands(
                Commands.defer(() -> {
                    // The return statement is required because of the curly braces
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
                                        // Do not flip this one, it does it in the command
                                        new AimAtHub3(hoodSubsystem, shooterSubsystem, turretSubsystem, swerveSubsystem, () -> FieldConstants.BLUE_HUB_POSE3D.toPose2d().getTranslation()),
                                        new AgitateWithIntake(intakeSubsystem)
                                )
                        )
                
                    );

                }, Set.of(swerveSubsystem, intakeSubsystem, shooterSubsystem, indexerSubsystem))
        );
    }
}