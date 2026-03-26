package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class AgitateWithIntake extends Command {

    IntakeSubsystem intakeSubsystem;
    Timer timer = new Timer();
    static double A = 5.0;   // travel amplitude (voltage)
    static double B = 2.0;   // buzz amplitude
    static double w = 0.5;   // carrier frequency
    static double b = 14.0;  // buzz frequency
    static double D = 0.8;   // dead band width

    private static double f(double x) {
        double sinWX = Math.sin(w * x);
        double cosWX = Math.cos(w * x);

        double rising  = A * sinWX * Math.max(0, Math.signum(cosWX));
        double falling = A * (Math.max(0, Math.abs(sinWX) - D) / (1 - D))
                * Math.signum(sinWX)
                * Math.max(0, -Math.signum(cosWX));

        return rising + falling;
    }

    private static double g(double x) {
        double sinWX = Math.sin(w * x);
        double cosWX = Math.cos(w * x);

        return B * (Math.max(0, D - Math.abs(sinWX)) / D)
                * Math.max(0, -cosWX)
                * Math.sin(b * x);
    }

    public static double voltage(double x) {
        return f(x) + g(x);
    }

    public AgitateWithIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public void initialize() {
        timer.restart();
    }

    public void execute() {

        timer.start();
        double voltage = voltage(timer.get());

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

