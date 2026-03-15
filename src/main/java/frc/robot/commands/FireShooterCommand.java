package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class FireShooterCommand extends Command {

    ShooterSubsystem shooterSubsystem;
    IndexerSubsystem indexerSubsystem;
    TurretSubsystem turretSubsystem;

    public FireShooterCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, TurretSubsystem turretSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        boolean ready = !turretSubsystem.getSnapBackState() &&  shooterSubsystem.shooterAtSetpoint();
        indexerSubsystem.setHopperWithPreferences();
        if (ready) {
            indexerSubsystem.setHopperWithPreferences();
            indexerSubsystem.setKickerWithPreferences();
        } else {
            indexerSubsystem.stopMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
    }

}
