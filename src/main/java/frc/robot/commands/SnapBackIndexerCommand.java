package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SnapBackIndexerCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;
    private final TurretSubsystem turretSubsystem;

    public SnapBackIndexerCommand(IndexerSubsystem indexerSubsystem, TurretSubsystem turretSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        this.turretSubsystem = turretSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        if (!turretSubsystem.getSnapBackState()) {
            indexerSubsystem.setHopperWithPreferences();
            indexerSubsystem.setKickerWithPreferences();
        }
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
    }

}
