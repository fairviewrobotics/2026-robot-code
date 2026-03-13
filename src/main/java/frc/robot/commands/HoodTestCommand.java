package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class HoodTestCommand extends Command {

    HoodSubsystem hoodSubsystem;

    public HoodTestCommand(HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        hoodSubsystem.setHoodWithPreferences();
    }

    public void end() {
        // hoodSubsystem.setHood(0.05);
    }

}
