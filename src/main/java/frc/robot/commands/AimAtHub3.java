package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.filter.LinearFilter;


import java.util.function.Supplier;

public class AimAtHub3 extends Command {
    HoodSubsystem hood;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    SwerveSubsystem swerve;
    Supplier<Translation2d> target;

    public AimAtHub3(HoodSubsystem hood, ShooterSubsystem shooter, TurretSubsystem turret, SwerveSubsystem swerve, Supplier<Translation2d> target){
        this.hood = hood;
        this.shooter = shooter;
        this.turret = turret;
        this.swerve = swerve;
        this.target = target;
        addRequirements(hood, shooter, turret);
        Preferences.initDouble("AimAtHub/SHOOTER_RPM_SCALAR", 1.0);
        Preferences.initDouble("AimAtHub/UNIVERSAL_SCALAR", 1.0);
        Preferences.initDouble("AimAtHub/PHASE_DELAY", 0.1);
    }


    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds fieldVel = swerve.getFieldVelocity();
        double phaseDelay = Preferences.getDouble("AimAtHub/PHASE_DELAY", 0.1);

        Pose2d robotAtRelease = currentPose.exp(new Twist2d(
                swerve.getRobotVelocity().vxMetersPerSecond * phaseDelay,
                swerve.getRobotVelocity().vyMetersPerSecond * phaseDelay,
                swerve.getRobotVelocity().omegaRadiansPerSecond * phaseDelay
        ));

        double robotRotationRad = currentPose.getRotation().getRadians();
        double omega = fieldVel.omegaRadiansPerSecond;

        double offsetX = ShootingConstants.TURRET_TRANSFORM_2D.getX();
        double offsetY = ShootingConstants.TURRET_TRANSFORM_2D.getY();

        double shooterFieldVelX = fieldVel.vxMetersPerSecond +
                (omega * (-offsetY * Math.cos(robotRotationRad) - offsetX * Math.sin(robotRotationRad)));
        double shooterFieldVelY = fieldVel.vyMetersPerSecond +
                (omega * (offsetX * Math.cos(robotRotationRad) - offsetY * Math.sin(robotRotationRad)));

        Logger.recordOutput("Shooter/EffectiveVelocityX", shooterFieldVelX);
        Logger.recordOutput("Shooter/EffectiveVelocityY", shooterFieldVelY);

        Translation2d shooterTranslation = robotAtRelease.transformBy(ShootingConstants.TURRET_TRANSFORM_2D).getTranslation();
        Translation2d targetTranslation = AllianceFlipUtil.apply(target.get());

        double shooterDistance = targetTranslation.getDistance(shooterTranslation);
        Translation2d virtualTarget = targetTranslation;

        for (int i = 0; i < 10; i++) {
            double timeOfFlight = shooter.getDistanceToShotTime(shooterDistance);

            virtualTarget = targetTranslation.minus(new Translation2d(
                    shooterFieldVelX * timeOfFlight,
                    shooterFieldVelY * timeOfFlight
            ));

            shooterDistance = shooterTranslation.getDistance(virtualTarget);
        }

        double baseRPM = shooter.getDistanceToRPMMap(shooterDistance);
        Rotation2d turretFieldAngle = virtualTarget.minus(shooterTranslation).getAngle();
        Rotation2d robotRelativeTurretAngle = turretFieldAngle.minus(currentPose.getRotation());

        double adjustedAngle = MathUtil.inputModulus(robotRelativeTurretAngle.plus(Rotation2d.fromDegrees(180)).getRadians(), 0, 2 * Math.PI);
        adjustedAngle = Math.abs(adjustedAngle);

        double universalScalar = Preferences.getDouble("AimAtHub/UNIVERSAL_SCALAR", 1.0);
        double finalRPM = baseRPM * universalScalar;

        if (shooterDistance >= 5.6) {
            hood.setHood(0.9);
        } else if (shooterDistance >= 2.4) {
            hood.setHood(0.6);
        } else if (shooterDistance >= 2.2) {
            hood.setHood(0.4);
        } else {
            hood.setHood(0.3);
        }

        shooter.setMotorRPM(finalRPM);
        turret.setTurret(adjustedAngle);

        Logger.recordOutput("RobotAtRelease", robotAtRelease);
        Logger.recordOutput("Shooter/VirtualTarget", virtualTarget);
        Logger.recordOutput("Shooter/ShooterDistance", shooterDistance);
        Logger.recordOutput("Shooter/OTFTargetRPM", finalRPM);
    }

    @Override
    public void end(boolean interrupted) {
        hood.setHood(0.05);
        shooter.stopMotors();
        turret.setVoltage(0.0);
    }

}
