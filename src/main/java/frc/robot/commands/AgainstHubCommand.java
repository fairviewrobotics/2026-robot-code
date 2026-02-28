package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.*;

public class AgainstHubCommand extends Command {

    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    public AgainstHubCommand(HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void execute() {
        turretSubsystem.setTurret(0.0);
        hoodSubsystem.setHood(ShootingConstants.HOOD_AGAINST_THE_HUB_ANGLE_DEGREES);
        shooterSubsystem.setMotorRPM(ShootingConstants.SHOOTER_AGAINST_THE_HUB_RPM);
    }

    public void end() {
        hoodSubsystem.setHood(ShootingConstants.HOOD_MAX_ANGLE_DEGREES);
        shooterSubsystem.stopMotors();
        turretSubsystem.setVoltage(0.0);
    }

}
