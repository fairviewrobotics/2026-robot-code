package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final double voltage;
    public RetractIntakeCommand(IntakeSubsystem intakeSubsystem, double voltage) {
        this.intakeSubsystem = intakeSubsystem;
        this.voltage = voltage;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        intakeSubsystem.setIntakeDeployMotorVoltage(voltage);
        intakeSubsystem.setIntakeRollerMotorVoltage(-voltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }
}
