package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.routines.DepotShoot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("DEPOT_SHOOT", new DepotShoot(swerveSubsystem, turretSubsystem));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}
