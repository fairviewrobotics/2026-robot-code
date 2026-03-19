package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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

import java.util.function.Supplier;

public class AimAtHub3NoTurret extends Command {

HoodSubsystem hood;
ShooterSubsystem shooter;
SwerveSubsystem swerve;
Supplier<Translation2d> target;

public AimAtHub3NoTurret(HoodSubsystem hood, ShooterSubsystem shooter, SwerveSubsystem swerve, Supplier<Translation2d> target) {
    this.hood = hood;
    this.shooter = shooter;
    this.swerve = swerve;
    this.target = target;
    addRequirements(hood, shooter);
        Preferences.initDouble("AimAtHub/SHOOTER_RPM_SCALAR", 1.0);
        Preferences.initDouble("AimAtHub/PHASE_DELAY", 0.1);
}


@Override
public void execute() {

    Pose2d currentPose = swerve.getPose();
    ChassisSpeeds fieldVel = swerve.getFieldVelocity();

    Pose2d robotAtRelease = currentPose.exp(new Twist2d(
            swerve.getRobotVelocity().vxMetersPerSecond * Preferences.getDouble("AimAtHub/PHASE_DELAY", 0.1),
            swerve.getRobotVelocity().vyMetersPerSecond * Preferences.getDouble("AimAtHub/PHASE_DELAY", 0.1),
            swerve.getRobotVelocity().omegaRadiansPerSecond * Preferences.getDouble("AimAtHub/PHASE_DELAY", 0.1)
    ));

    Logger.recordOutput("RobotAtRelease", robotAtRelease);

    Translation2d shooterTranslation = robotAtRelease.transformBy(ShootingConstants.TURRET_TRANSFORM_2D).getTranslation();
    Translation2d targetTranslation = AllianceFlipUtil.apply(target.get());
    double shooterDistance = targetTranslation.getDistance(shooterTranslation);
    Translation2d virtualTarget = targetTranslation;
    double timeOfFlight;

    for (int i = 0; i < 10; i++) {
        timeOfFlight = shooter.getDistanceToShotTimeLeft(shooterDistance);

        virtualTarget = targetTranslation.plus(new Translation2d(
                fieldVel.vxMetersPerSecond * timeOfFlight,
                fieldVel.vyMetersPerSecond * timeOfFlight
        ));

        shooterDistance = shooterTranslation.getDistance(virtualTarget);
    }

    double RPM = shooter.getDistanceToRPMMapLeft(shooterDistance);
//        double hoodAngle = hood.getHoodSetpoint(shooterDistance);

    Rotation2d turretFieldAngle = virtualTarget.minus(shooterTranslation).getAngle();

    Rotation2d robotRelativeTurretAngle = turretFieldAngle.minus(currentPose.getRotation());
    double adjustedAngle = MathUtil.inputModulus(robotRelativeTurretAngle.plus(Rotation2d.fromDegrees(180)).getRadians(), 0, 2 * Math.PI);
    adjustedAngle = Math.abs(adjustedAngle);
    Logger.recordOutput("Turret/RobotTurretAngle", robotRelativeTurretAngle);
    Logger.recordOutput("Turret/AdjustedAngle", adjustedAngle);
    Logger.recordOutput("Shooter/ShooterDistance", shooterDistance);
    Logger.recordOutput("Shooter/OTFTargetRPM", RPM);

    if (shooterDistance >= 5.6) {
        hood.setHood(0.9);
    } else if (shooterDistance >= 2.4) {
        hood.setHood(0.6);
    } else if (shooterDistance >= 2.2) {
        hood.setHood(0.4);
    } else {
        hood.setHood(0.25);
    }
    shooter.setMotorRPM(RPM);
}

@Override
public void end(boolean interrupted) {
    hood.setHood(0.05);
    shooter.stopMotors();
}

}
