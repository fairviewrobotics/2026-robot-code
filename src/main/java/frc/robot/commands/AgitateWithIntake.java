package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AgitateWithIntake extends Command {

    IntakeSubsystem intakeSubsystem;
    Timer timer = new Timer();
    double amplitude = 4.0; // Max voltage
    double frequency = 0.5; // Oscillations per second (Hz)


    public AgitateWithIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        Preferences.initDouble("Agitation/Frequency", frequency);
        Preferences.initDouble("Agitation/Amplitude", amplitude);
        Preferences.initDouble("Agitation/Power", 0);
    }

    public void initialize() {
        timer.restart();
    }

    public void execute() {
        double freq = Preferences.getDouble("Agitation/Frequency", frequency);
        double amp = Preferences.getDouble("Agitation/Amplitude", amplitude);
        double pow = Preferences.getDouble("Agitation/Power", 0);

//        double wave = Math.sin(2 * Math.PI * freq * timer.get());

//        double wave = Math.cos(timer.get() + Math.sin(timer.get()));

        double wave = 2 * Math.pow(Math.abs(Math.sin(timer.get() * freq)), pow) - 1;

        double voltage = (amp * wave);

        voltage = Math.max(-12.0, Math.min(12.0, voltage));
        intakeSubsystem.setIntakeDeployMotorVoltage(voltage);

        intakeSubsystem.setIntakeDeployMotorVoltage(voltage);
        intakeSubsystem.setIntakeRollerMotor(0.1153846154);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotors();
        timer.stop();
    }

}
