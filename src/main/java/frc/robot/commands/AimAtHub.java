package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TunableNumber;

public class AimAtHub extends Command {

    private Pose2d currentPose;
    private Pose3d targetPose;
    private SwerveSubsystem swerveSubsystem;
    private RobotState robotState;
    private Pose2d shooterVelocity;
    private TurretSubsystem turret;



    public AimAtHub(SwerveSubsystem swerveSubsystem, RobotState robotState, Pose2d currentPose, Pose3d targetPose, Pose2d shooterVelocity, TurretSubsystem turret) {
        this.swerveSubsystem = swerveSubsystem;
        this.robotState = robotState;
        this.currentPose = currentPose;
        this.targetPose = targetPose;
        this.shooterVelocity = shooterVelocity;
        this.turret = turret;
    }

    public double angel(Pose2d pose) {
        return Math.atan2(pose.getX(), pose.getY());
    }

    public double distance(Pose2d pose) {
        return Math.hypot(pose.getX(), pose.getY());
    }

    public double turretRotationAngel(double turretAngle, double robotAngle, double targetAngle) {
        return turretAngle * -1 + robotAngle * -1 +  targetAngle;
    }

    public Pose2d getRelative2d(Pose2d pose0, Pose2d pose1,  Pose2d pose2, double t, int n, double vMag) {
        Pose2d pose = new Pose2d( new Translation2d((pose0.getX()-(pose1.getX() + pose2.getX() * t)),(pose0.getY()-(pose1.getY() + pose2.getY() * t))), pose0.getRotation());
        for(int i = 0; i < n; i++) {
            t = distance(pose)/vMag;
            pose = new Pose2d( new Translation2d((pose0.getX()-(pose1.getX() + pose2.getX() * t)),(pose0.getY()-(pose1.getY() + pose2.getY() * t))), pose0.getRotation());
        }
        return pose;
    }
    public Pose3d getRelative3d(Pose3d pose0, Pose2d pose1,  Pose2d pose2, double t, int n, double vMag) {
        Pose3d pose = new Pose3d( new Translation3d((pose0.getX()-(pose1.getX() + pose2.getX() * t)),(pose0.getY()-(pose1.getY() + pose2.getY() * t)), pose0.getZ()), pose0.getRotation());
        for(int i = 0; i < n; i++) {
            t= distance(pose.toPose2d())/vMag;
            pose = new Pose3d( new Translation3d((pose0.getX()-(pose1.getX() + pose2.getX() * t)),(pose0.getY()-(pose1.getY() + pose2.getY() * t)), pose0.getZ()), pose0.getRotation());
        }
        return pose;
    }

    public Pose2d subtract(Pose2d pose0, Pose2d pose1) {
        return new Pose2d(new Translation2d(pose0.getX()-pose1.getX(), pose0.getY()-pose1.getY()), pose0.getRotation());
    }

    public double shootingAngle(Pose3d relitivePos, double v, double t){
        double dx = relitivePos.getX();
        double dy = relitivePos.getY();
        double dz = relitivePos.getZ();
        double d = Math.hypot(dx, dy);
        double epsilon = 0.01;
        if (v *t + epsilon < d) {
            return -1;
        }
        double z = (dz + 0.5 * 9.8 * t *t) / (v*t);
        double x = (d) / (v*t);
        return Math.atan2(x,z);
    }

    @Override
    public void execute() {
        Pose2d targetPos2d = targetPose.toPose2d();
        Pose2d robotVelocity = new Pose2d(
                new Translation2d(
                        swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                        swerveSubsystem.getFieldVelocity().vyMetersPerSecond),
                new Rotation2d(swerveSubsystem.getFieldVelocity().omegaRadiansPerSecond));

        Pose2d horizontalVector = subtract(targetPos2d, currentPose);
        double vMag = distance(shooterVelocity);
        double time = distance(horizontalVector)/vMag;

        Pose2d relativePose2d = getRelative2d(targetPos2d, currentPose, robotVelocity , time, 3, vMag);
        Pose3d relativePose3d = getRelative3d(targetPose, currentPose, robotVelocity, time, 3, vMag);
        double d = distance(relativePose2d);
        time = d/vMag;

        turret.setTurret(angel(relativePose2d) + currentPose.getRotation().getRadians() * -1);
        double shooterAngle = shootingAngle(relativePose3d, vMag, time);
    }


}

