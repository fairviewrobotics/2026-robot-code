package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.routines.BlueTrenchLeft;
import frc.robot.autonomous.routines.TestAuto;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(SwerveSubsystem swerveSubsystem) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("TEST AUTO", new TestAuto(swerveSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH LEFT", new BlueTrenchLeft(swerveSubsystem));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}
