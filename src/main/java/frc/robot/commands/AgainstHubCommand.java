package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.*;

public class AgainstHubCommand extends Command {

    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;

    public AgainstHubCommand(HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(hoodSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        hoodSubsystem.setHood(ShootingConstants.HOOD_AGAINST_THE_HUB_ANGLE_DEGREES);
        shooterSubsystem.setMotorRPM(ShootingConstants.SHOOTER_AGAINST_THE_HUB_RPM);
    }

    public void end() {
        hoodSubsystem.setHood(0.05);
        shooterSubsystem.stopMotors();
    }

}
