package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private double voltage;
    public IntakeCommand(IntakeSubsystem intakeSubsystem, double voltage) {
        this.intakeSubsystem = intakeSubsystem;
        this.voltage = voltage;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakeSubsystem.setIntakeRollerMotorWithPreferences();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

}
