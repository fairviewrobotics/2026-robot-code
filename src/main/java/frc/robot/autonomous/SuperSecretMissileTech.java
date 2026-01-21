package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}
