package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

//Example usage of Shooter Subsystem, not in Robot Container

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double topShooterRPM;
    private double bottomShooterRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double topShooterRPM, double bottomShooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.topShooterRPM = topShooterRPM;
        this.bottomShooterRPM = bottomShooterRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setTopMotorRPM(topShooterRPM);
        shooterSubsystem.setBottomMotorRPM(bottomShooterRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setMotorRPM(0);
    }

}