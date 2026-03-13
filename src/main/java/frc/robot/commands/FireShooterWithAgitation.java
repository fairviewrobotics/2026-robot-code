package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FireShooterWithAgitation extends Command {

    IntakeSubsystem intakeSubsystem;
    Timer timer = new Timer();
    double amplitude = 3.5; // Max voltage
    double frequency = 1.25; // Oscillations per second (Hz)


    public FireShooterWithAgitation(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public void initialize() {
        timer.restart();
    }

    public void execute() {
        double voltage = amplitude * Math.sin(2 * Math.PI * frequency * timer.get());
        intakeSubsystem.setIntakeDeployMotorVoltage(voltage);
        intakeSubsystem.setIntakeRollerMotor(0.1153846154);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        timer.stop();
    }

}
