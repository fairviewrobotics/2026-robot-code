package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Drop down intake, run roller motors
        // 2 volts CCW
        double voltage = -2;

        // Added hard stop calculation logic
        intakeSubsystem.swapIntakeState();

        if (intakeSubsystem.getIntakeState() == IntakeSubsystem.IntakeState.RETRACTED) {
            intakeSubsystem.setIntakeDeployMotorVoltage(voltage);
        }
        intakeSubsystem.setIntakeRollerMotorWithPreferences();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
    }

}
