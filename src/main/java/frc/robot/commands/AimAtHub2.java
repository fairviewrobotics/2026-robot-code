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
import org.littletonrobotics.junction.Logger;

// IMPORTANT READ THIS
// x,y,z = forward/back, left/right, up/down

public class AimAtHub2 extends Command {
    // private Pose3d currentPose;
    private Pose3d targetPose;
    private SwerveSubsystem swerveSubsystem;
    private TurretSubsystem turretSubsystem;
    private double time;
    private final double g = -9.81;
    private Translation2d robotAceleration;

    public AimAtHub2(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem, boolean red, double time, Translation2d robotAceleration){
        this.swerveSubsystem = swerveSubsystem;
        this.turretSubsystem = turretSubsystem;
        /*this.currentPose = new Pose3d(this.swerveSubsystem.getPose().getX(),
                this.swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(this.swerveSubsystem.getPose().getRotation()));
                // TODO: height of turret!
         */
        if (red) {
            this.targetPose = FieldConstants.RED_HUB_POSE3D;
        }
        else {
            this.targetPose = FieldConstants.BLUE_HUB_POSE3D;
        }
        this.time = time;
        this.robotAceleration = robotAceleration;
    }

    @Override
    public void execute() {
        turretSubsystem.setTurret(turretAngle(new Pose3d(swerveSubsystem.getPose().getX(),
                swerveSubsystem.getPose().getY(),
                0.0,
                new Rotation3d(swerveSubsystem.getPose().getRotation()))));
        Logger.recordOutput("Target Pose", targetPose);
    }

    private static Pose3d getTurretPose3d(Pose3d currentPose, Translation3d turretOffset) {
        Translation3d rotatedOffset = turretOffset.rotateBy(currentPose.getRotation());
        return new Pose3d(currentPose.getTranslation().plus(rotatedOffset), currentPose.getRotation());
    }

    private static Pose3d getRelativePose3d(Pose3d targetPose, Pose3d turretPose, Translation2d robotVelocity, Translation2d robotAceleration, double time) {
        return new Pose3d(targetPose.getX() - (turretPose.getX() + (robotVelocity.getX()* time) + (0.5 * robotAceleration.getX() * time * time )),
                targetPose.getY() - (turretPose.getY() + (robotVelocity.getY() * time) + (0.5 * robotAceleration.getY() * time * time)),
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


    public double turretAngle(Pose3d currentPose){
        Translation3d turretOffset = new Translation3d(-0.0508, -0.381, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration, time);
        double turretAngle = Math.atan2(relativePose.getY(),relativePose.getX());
        Logger.recordOutput("Turret/evenbetterturretangle", turretAngle);
        turretAngle +=swerveSubsystem.getPose().getRotation().getRadians() * -1;
        turretAngle = turretAngle % (Math.PI * 2);
        if(turretAngle < 0){
            turretAngle += Math.PI * 2;
        }
        return turretAngle;
    }


    public double shootingAngle(Pose3d currentPose){
        Translation3d turretOffset = new Translation3d(0.5, 0.0, 0.0);// TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration, time);
        double[] velocity =  getVelocity(relativePose, time, g);
        return  Math.atan2(velocity[1], velocity[0]);
    }
    public double[] shootingVelocity(Pose3d currentPose){
        Translation3d turretOffset = new Translation3d(-0.0508, -0.381, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration time);
        return getVelocity(relativePose, time, g);
    }

    @Override
    public void initialize() {
        // turretSubsystem.resetPID();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setVoltage(0.0);
    }

}