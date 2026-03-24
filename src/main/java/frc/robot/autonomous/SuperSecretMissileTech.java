package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.routines.BlueDepotLeft;
import frc.robot.autonomous.routines.BlueTrenchLeft;
import frc.robot.autonomous.routines.BlueTrenchLeftSelfPass;
import frc.robot.autonomous.routines.BlueTrenchLeftWithDepot;
import frc.robot.autonomous.routines.BlueTrenchLeftWithPreload;
import frc.robot.autonomous.routines.BlueTrenchRight;
import frc.robot.autonomous.routines.BlueTrenchRightWithPreload;
import frc.robot.autonomous.routines.PreloadDump;
import frc.robot.autonomous.routines.PreloadDumpRight;
import frc.robot.autonomous.routines.TestAuto;
import frc.robot.subsystems.*;

public class SuperSecretMissileTech {

    private final SendableChooser<SequentialCommandGroup> superSecretMissileTech = new SendableChooser<>();

    public SuperSecretMissileTech(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, TurretSubsystem turretSubsystem, ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem) {
        superSecretMissileTech.setDefaultOption("NOTHING", new SequentialCommandGroup());
        superSecretMissileTech.addOption("TEST AUTO", new TestAuto(swerveSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH LEFT", new BlueTrenchLeft(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH LEFT WITH PRELOAD", new BlueTrenchLeftWithPreload(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("PRELOAD DUMP", new PreloadDump(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("PRELOAD DUMP RIGHT", new PreloadDumpRight(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE DEPOT LEFT", new BlueDepotLeft(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH LEFT WITH DEPOT", new BlueTrenchLeftWithDepot(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH RIGHT", new BlueTrenchRight(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH RIGHT WITH PRELOAD", new BlueTrenchRightWithPreload(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        superSecretMissileTech.addOption("BLUE TRENCH LEFT SELF PASS", new BlueTrenchLeftSelfPass(swerveSubsystem, intakeSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem, indexerSubsystem));
        SmartDashboard.putData("Autonomous Selector", superSecretMissileTech);
    }

    public SequentialCommandGroup getSelected() {
        return superSecretMissileTech.getSelected();
    }

}
