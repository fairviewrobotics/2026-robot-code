package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final double intakeVoltage;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, double intakeVoltage) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeVoltage = intakeVoltage;
        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // intakeSubsystem.setVoltage(intakeVoltage);
        // intakeSubsystem.setIndexerVoltage(intakeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

}
