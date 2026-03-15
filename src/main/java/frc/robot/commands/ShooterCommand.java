package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

//Example usage of Shooter Subsystem, not in Robot Container

public class ShooterCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final TurretSubsystem turretSubsystem;
    private double topShooterRPM;
    private double bottomShooterRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, double topShooterRPM, double bottomShooterRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.topShooterRPM = topShooterRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        double RPM = Preferences.getDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM);
        double shooterSetpoint = RPM * Preferences.getDouble("AimAtHub/SHOOTER_RPM_SCALAR", 1.0);
        shooterSubsystem.setMotorRPM(shooterSetpoint);
        // shooterSubsystem.setMotorRPM(bottomShooterRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopMotors();
    }

}