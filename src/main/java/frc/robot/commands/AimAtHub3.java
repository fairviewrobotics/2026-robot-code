package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAtHub3 extends Command {
    HoodSubsystem hood;
    ShooterSubsystem shooter;
    SwerveSubsystem swerve;
    Translation2d target;

    public AimAtHub3(HoodSubsystem hood, ShooterSubsystem shooter, SwerveSubsystem swerve, Translation2d target){
        this.hood = hood;
        this.shooter = shooter;
        this.swerve = swerve;
        this.target = target;
    }
    @Override
    public void execute(){
        Pose2d currentPose = swerve.getPose();
        Pose2d futerPose = currentPose.exp(new Twist2d(
                swerve.getFieldVelocity().vxMetersPerSecond * 0.1,
                swerve.getFieldVelocity().vyMetersPerSecond * 0.1,
                swerve.getFieldVelocity().omegaRadiansPerSecond * 0.1
        ));
        Pose2d shooterPose = futerPose.transformBy(ShootingConstants.TURRET_OFFSET);
        double shooterDistance = target.getDistance(shooterPose.getTranslation());
        double shootTime = shooter.getDistanceToShotTime(shooterDistance);
        double RPM = shooter.getDistanceToRPMMap(shooterDistance);
        double hoodAngle = hood.getHoodSetpoint(shooterDistance);




    }
}
