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

    public TrenchRightCommand(HoodSubsystem hoodSubsystem, ShooterSubsystem shooterSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(hoodSubsystem, shooterSubsystem);
    }

    @Override
    public void execute() {
        hoodSubsystem.setHood(ShootingConstants.HOOD_TRENCH_ANGLE);
        shooterSubsystem.setMotorRPM(ShootingConstants.SHOOTER_TRENCH_RPM);
    }

    public void end() {
        hoodSubsystem.setHood(0.05);
        shooterSubsystem.stopMotors();
    }


}
