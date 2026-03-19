package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class TurretVoltageCommand extends Command {

    TurretSubsystem turret;

    public TurretVoltageCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    public void execute() {

    }

}
