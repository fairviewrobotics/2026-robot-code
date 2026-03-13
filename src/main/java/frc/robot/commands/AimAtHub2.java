package frc.robot.commands;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
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

import java.util.Optional;

// IMPORTANT READ THIS
// x,y,z = forward/back, left/right, up/down

public class AimAtHub2 extends Command {
    // private Pose3d currentPose;
    private Pose3d targetPose;
    private SwerveSubsystem swerveSubsystem;
    private TurretSubsystem turretSubsystem;
    private double time;
    private final double g = -9.81;
    private TimeInterpolatableBuffer<Translation2d> velocityBuffer = TimeInterpolatableBuffer.createBuffer(1);

    public AimAtHub2(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem, boolean red, double time){
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
    }

    @Override
    public void execute() {
        velocityBuffer.addSample(Timer.getFPGATimestamp(), new Translation2d(swerveSubsystem.getFieldVelocity().vxMetersPerSecond, swerveSubsystem.getFieldVelocity().vyMetersPerSecond));
        Optional<Translation2d> velOptional =
                velocityBuffer.getSample(Timer.getFPGATimestamp() - 1);
        Translation2d oldVelocity = new Translation2d(swerveSubsystem.getFieldVelocity().vxMetersPerSecond, swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        if (velOptional.isPresent()) {
            oldVelocity = velOptional.get();
        }
        Translation2d acceleration = new Translation2d(swerveSubsystem.getFieldVelocity().vxMetersPerSecond - oldVelocity.getX(), swerveSubsystem.getFieldVelocity().vyMetersPerSecond - oldVelocity.getY());
        turretSubsystem.setTurret(turretAngle(new Pose3d(swerveSubsystem.getPose().getX(),
                        swerveSubsystem.getPose().getY(),
                        0.0,
                        new Rotation3d(swerveSubsystem.getPose().getRotation())),
                acceleration));
        acceleration = new Translation2d(0,0);
        Logger.recordOutput("Turret/Target Pose", targetPose);
        Translation2d ballVelocity =
                shootingVelocity(
                        new Pose3d(swerveSubsystem.getPose().getX(),
                                swerveSubsystem.getPose().getY(),
                                0.0,
                                new Rotation3d(swerveSubsystem.getPose().getRotation())),
                        acceleration);
        Logger.recordOutput("Turret/Ball velocity", ballVelocity);
        double hoodAngle =
                shootingAngle(
                        new Pose3d(swerveSubsystem.getPose().getX(),
                                swerveSubsystem.getPose().getY(),
                                0.0,
                                new Rotation3d(swerveSubsystem.getPose().getRotation())),
                        acceleration);
        Logger.recordOutput("Turret/Hood angle", hoodAngle);
        Pose3d robot =getEstimatedPose3d(new Pose3d(swerveSubsystem.getPose()),new Translation2d(swerveSubsystem.getFieldVelocity().vxMetersPerSecond,swerveSubsystem.getFieldVelocity().vyMetersPerSecond),acceleration,time);
        double turretAngle = turretAngle(robot, acceleration);
        double horizontalDist = ballVelocity.getX() * time;
        turretAngle += swerveSubsystem.getPose().getRotation().getRadians();
        Logger.recordOutput("Turret/Ball Pose",
                new Translation3d(
                        robot.getX() + horizontalDist * Math.cos(turretAngle),
                        robot.getY() + horizontalDist * Math.sin(turretAngle),
                        robot.getZ() + ballVelocity.getY() * time + 0.5 * g * time * time));
    }

    private static Pose3d getTurretPose3d(Pose3d currentPose, Translation3d turretOffset) {
        Translation3d rotatedOffset = turretOffset.rotateBy(currentPose.getRotation());
        return new Pose3d(currentPose.getTranslation().plus(rotatedOffset), currentPose.getRotation());
    }

    private static Pose3d getRelativePose3d(Pose3d targetPose, Pose3d turretPose, Translation2d robotVelocity, Translation2d robotAcceleration, double time) {
        return new Pose3d(targetPose.getX() - (turretPose.getX() + (robotVelocity.getX()* time) + (0.5 * robotAcceleration.getX() * time * time )),
                targetPose.getY() - (turretPose.getY() + (robotVelocity.getY() * time) + (0.5 * robotAcceleration.getY() * time * time)),
                targetPose.getZ() - turretPose.getZ(),
                targetPose.getRotation());
    }

    private static Pose3d getEstimatedPose3d(Pose3d turretPose, Translation2d robotVelocity, Translation2d robotAcceleration, double time){
        return new Pose3d((turretPose.getX() + (robotVelocity.getX()* time) + (0.5 * robotAcceleration.getX() * time * time )),
                (turretPose.getY() + (robotVelocity.getY() * time) + (0.5 * robotAcceleration.getY() * time * time)),
                0.0,
                turretPose.getRotation());
    }

    private static Translation2d getVelocity(Pose3d relativePose, double time, double g) {
        double dx = relativePose.getX();
        double dz = relativePose.getZ();

        double vx = dx/time;
        double vz = (dz - 0.5 * g * time * time)/time;
        Translation2d velocity = new Translation2d(vx,vz);
        return velocity;
    }


    public double turretAngle(Pose3d currentPose,Translation2d robotAceleration){
        Translation3d turretOffset = new Translation3d(-0.127, 0.0, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration, time);
        Logger.recordOutput("Turret/relative Pose", relativePose);
        Logger.recordOutput("Turret/estimated Pose", getEstimatedPose3d(turretPose,robotVelocity,robotAceleration,time));
        double turretAngle = Math.atan2(relativePose.getY(),relativePose.getX());
        Logger.recordOutput("Turret/evenbetterturretangle", turretAngle);
        turretAngle +=swerveSubsystem.getPose().getRotation().getRadians() * -1;
        turretAngle = turretAngle % (Math.PI * 2);
        if(turretAngle < 0){
            turretAngle += Math.PI * 2;
        }
        return turretAngle;
    }


    public double shootingAngle(Pose3d currentPose, Translation2d robotAceleration){
        Translation3d turretOffset = new Translation3d(0.5, 0.0, 0.0);// TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration, time);
        Translation2d velocity =  getVelocity(relativePose, time, g);
        return  Math.atan2(velocity.getY(), velocity.getX());
    }
    public Translation2d shootingVelocity(Pose3d currentPose, Translation2d robotAceleration){
        Translation3d turretOffset = new Translation3d(-0.0508, -0.381, 0.0); // TODO: get offset from cad
        Translation2d robotVelocity = new Translation2d(
                swerveSubsystem.getFieldVelocity().vxMetersPerSecond,
                swerveSubsystem.getFieldVelocity().vyMetersPerSecond);
        Pose3d turretPose = getTurretPose3d(currentPose, turretOffset);
        Pose3d relativePose = getRelativePose3d(targetPose,turretPose, robotVelocity, robotAceleration, time);
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
