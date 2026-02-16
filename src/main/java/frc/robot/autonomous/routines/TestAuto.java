package frc.robot.autonomous.routines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoint;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class TestAuto extends SequentialCommandGroup {
    public TestAuto(SwerveSubsystem swerveSubsystem) {
        setName("TEST AUTO");
        addRequirements(swerveSubsystem);
        addCommands(
            new DriveToPoint(swerveSubsystem, FieldConstants.CARPET_POINT, 0.5),
            new DriveToPoint(swerveSubsystem, FieldConstants.ODOMETRY_RESET_POINT, 0.5),
            new DriveToPoint(swerveSubsystem, FieldConstants.CARPET_POINT2, 0.5)
        );
    }
}
