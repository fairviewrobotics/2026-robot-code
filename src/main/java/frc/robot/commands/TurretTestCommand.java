package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class TurretTestCommand extends Command {

    SwerveSubsystem swerveSubsystem;
    TurretSubsystem turretSubsystem;
    Pose2d targetPose;
    Pose2d currentPose;

    public TurretTestCommand(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem, Pose2d targetPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        turretSubsystem.resetPID();
    }

    @Override
    public void execute() {

            Pose2d robotPose = swerveSubsystem.getPose();

            // 1. Calculate the turret's actual position on the field
            // This rotates the offset by the robot's heading and adds it to the robot's X/Y
            Translation2d turretGlobalPosition = robotPose.getTranslation()
                    .plus(ShootingConstants.TURRET_OFFSET2D.rotateBy(robotPose.getRotation()));

            // 2. Calculate the angle from the TURRET to the target
            Rotation2d angleToTarget = targetPose.getTranslation()
                    .minus(turretGlobalPosition)
                    .getAngle();

            // 3. Convert to local turret coordinates
            // We subtract the robot's heading because the turret motor
            // usually operates relative to the chassis.
            double targetAngle = angleToTarget.minus(robotPose.getRotation()).getRadians();

            double turretTargetAngle = MathUtil.angleModulus(targetAngle);

            turretSubsystem.setTurret(turretTargetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setVoltage(0.0);
    }

}
