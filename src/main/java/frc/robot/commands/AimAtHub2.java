package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.utils.TunableNumber;

// IMPORTANT READ THIS
// x,y,z = forward/back, left/right, up/down

public class AimAtHub2 extends Command {
    private Pose3d currentPose;
    private Pose3d targetPose;
    private SwerveSubsystem swerveSubsystem;
    private double time;
    private final double g = -9.81;

    public AimAtHub2(SwerveSubsystem swerveSubsystem, boolean red, double time){
        this.swerveSubsystem = swerveSubsystem;
        this.currentPose = new Pose3d(this.swerveSubsystem.getPose().getX(),
                this.swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(this.swerveSubsystem.getPose().getRotation())); // TODO: height of turret!
        if (red) {
            this.targetPose = FieldConstants.RED_HUB_POSE3D;
        }
        else {
            this.targetPose = FieldConstants.BLUE_HUB_POSE3D;
        }
        this.time = time;
    }

    private static Pose3d getTurretPose3d(Pose3d currentPose, Translation3d turretOffset) {
        turretOffset.rotateBy(currentPose.getRotation());
        return new Pose3d(currentPose.getTranslation().plus(turretOffset), currentPose.getRotation());
    }

    private static Pose3d getRelativePose3d(Pose3d targetPose, Pose3d turretPose, Translation2d robotVelocity, double time) {
        return new Pose3d(targetPose.getX() - (turretPose.getX() + (robotVelocity.getX()* time)),
                targetPose.getY() - (turretPose.getY() + (robotVelocity.getY() * time)),
                targetPose.getZ() - turretPose.getZ(),
                targetPose.getRotation());
    }

    private static double[] getVelocity(Pose3d relativePose, double time, double g) {
        double dx = relativePose.getX();
        double dz = relativePose.getZ();

        double vx = dx/time;
        double vz = (dz - 0.5 * g * time * time)/time;
        double[] velocity = {vx, vz};
        return velocity;
    }


    public double turretAngle(){
        Translation3d turretOffset = new Translation3d(0.5, 0.0, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                        swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                        swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, time);
        double turretAngle = Math.atan2(relativePose.getY(), relativePose.getX());
        if(turretAngle < 0){
            turretAngle += Math.PI*2;
        }
        return turretAngle;
    }

    public double shootingAngle(){
        Translation3d turretOffset = new Translation3d(0.5, 0.0, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, time);
        double[] velocity =  getVelocity(relativePose, time, g);
        return  Math.atan2(velocity[1], velocity[0]);
    }
    public double[] shootingVelocity(){
        Translation3d turretOffset = new Translation3d(0.5, 0.0, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, time);
        return getVelocity(relativePose, time, g);
    }


}
