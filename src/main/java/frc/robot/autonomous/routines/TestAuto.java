package frc.robot.autonomous.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import swervelib.SwerveDrive;

import java.util.Set;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(SwerveSubsystem swerveSubsystem) {
        setName("TEST AUTO");
        addRequirements(swerveSubsystem);
        addCommands(
            Commands.defer(() -> {
                return new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        Pose2d startPose = AllianceFlipUtil.apply(FieldConstants.START_POINT);
                        swerveSubsystem.resetOdometry(startPose);
                    }),
                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.TOP_RIGHT_POINT), 0.75),
                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BOTTOM_RIGHT_POINT), 0.75),
                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BOTTOM_LEFT_POINT), 0.75),
                    new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.START_POINT), 0.75)
                );
            }, Set.of(swerveSubsystem))
        );
    }
}
