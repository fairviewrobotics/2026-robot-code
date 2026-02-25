package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.DriveToPointContinuous;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * Left relative to driver
 */

public class BlueTrenchLeft extends SequentialCommandGroup {
    public BlueTrenchLeft(SwerveSubsystem swerveSubsystem) {
        setName("BLUE TRENCH LEFT");
        addRequirements(swerveSubsystem);
        addCommands(
                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.75),
                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_PICKUP_END), 0.5),
                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT_TRANSITION_PICKUP), 0.75),
                new DriveToPointContinuous(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT), 0.5),
                new DriveToPoint(swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_AUTO_SHOOT_LEFT_POINT), 1.0)
        );
    }
}
