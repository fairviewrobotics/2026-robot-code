package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends Command {

    private final IndexerSubsystem indexerSubsystem;

    public IndexerCommand(IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }

    @Override
    public void execute() {
        indexerSubsystem.setHopperWithPreferences();
        indexerSubsystem.setKickerWithPreferences();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopMotors();
    }

}
