package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class CornerLeftCommand extends Command {

    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    public CornerLeftCommand(HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void execute() {
        turretSubsystem.setTurret(Units.degreesToRadians(ShootingConstants.TURRET_CORNER_LEFT_ANGLE_DEGREES));
        hoodSubsystem.setHood(ShootingConstants.HOOD_CORNER_ANGLE);
        shooterSubsystem.setMotorRPM(ShootingConstants.SHOOTER_CORNER_RPM);
    }

    public void end() {
        hoodSubsystem.setHood(ShootingConstants.HOOD_MAX_ANGLE_DEGREES);
        shooterSubsystem.stopMotors();
        turretSubsystem.setVoltage(0.0);
    }


}
