package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAtHub3 extends Command {
    HoodSubsystem hood;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    SwerveSubsystem swerve;
    Translation2d target;

    public AimAtHub3(HoodSubsystem hood, ShooterSubsystem shooter, TurretSubsystem turret, SwerveSubsystem swerve, Translation2d target){
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.swerve = swerve;
        this.target = target;
    }
    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds fieldVel = swerve.getFieldVelocity();

        Pose2d robotAtRelease = currentPose.exp(new Twist2d(
                swerve.getRobotVelocity().vxMetersPerSecond * 0.1,
                swerve.getRobotVelocity().vyMetersPerSecond * 0.1,
                swerve.getRobotVelocity().omegaRadiansPerSecond * 0.1
        ));

        Translation2d shooterTranslation = robotAtRelease.transformBy(ShootingConstants.TURRET_OFFSET).getTranslation();

        double shooterDistance = target.getDistance(shooterTranslation);
        Translation2d virtualTarget = target;
        double timeOfFlight;

        for (int i = 0; i < 10; i++) {
            timeOfFlight = shooter.getDistanceToShotTime(shooterDistance);

            virtualTarget = target.minus(new Translation2d(
                    fieldVel.vxMetersPerSecond * timeOfFlight,
                    fieldVel.vyMetersPerSecond * timeOfFlight
            ));

            shooterDistance = shooterTranslation.getDistance(virtualTarget);
        }

        double RPM = shooter.getDistanceToRPMMap(shooterDistance);
        double hoodAngle = hood.getHoodSetpoint(shooterDistance);

        Rotation2d turretFieldAngle = virtualTarget.minus(shooterTranslation).getAngle();

        Rotation2d robotRelativeTurretAngle = turretFieldAngle.minus(currentPose.getRotation());

        hood.setHood(hoodAngle);
        shooter.setMotorRPM(RPM);
        turret.setTurret(robotRelativeTurretAngle.getRadians());
    }

    public void end() {
        // Lowest point of hood, highest exit angle
        hood.setHood(ShootingConstants.HOOD_MAX_ANGLE_DEGREES);
        shooter.setMotorRPM(0);
        // turret.setTurret(0);
    }
}
