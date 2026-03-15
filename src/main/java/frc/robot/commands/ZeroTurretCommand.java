package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

public class ZeroTurretCommand extends Command {

    private final TurretSubsystem turretSubsystem;

    public ZeroTurretCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (turretSubsystem.getLimitSwitch()) {
            turretSubsystem.setVoltage(-1.0);
        } else {
            turretSubsystem.setVoltage(0.0);
        }
    }

    @Override
    public boolean isFinished() {
        return !turretSubsystem.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setVoltage(0.0);

        // Only perform the zeroing logic if we actually hit the switch
        if (!interrupted) {
            turretSubsystem.zeroTurret();
        }
    }
}