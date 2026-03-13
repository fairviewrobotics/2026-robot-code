package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TrenchRightCommand extends Command {

    HoodSubsystem hoodSubsystem;
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;

    public TrenchRightCommand(HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void execute() {
        turretSubsystem.setTurret(Units.degreesToRadians(ShootingConstants.TURRET_TRENCH_RIGHT_ANGLE_DEGREES));
        hoodSubsystem.setHood(ShootingConstants.HOOD_TRENCH_ANGLE);
        shooterSubsystem.setMotorRPM(ShootingConstants.SHOOTER_TRENCH_RPM);
    }

    public void end() {
        hoodSubsystem.setHood(ShootingConstants.HOOD_MAX_ANGLE_DEGREES);
        shooterSubsystem.stopMotors();
        turretSubsystem.setVoltage(0.0);
    }


}
