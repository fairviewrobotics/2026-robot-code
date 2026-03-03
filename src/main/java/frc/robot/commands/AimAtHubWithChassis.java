package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * Command to aim the center of the robot's front face at the hub target
 * while allowing manual joystick control for translation.
 * This serves as a backup aiming method if the turret is non-functional or the vision pose estimation goes to crap.
 *
 * x,y,z = forward/back, left/right, up/down
 */

public class AimAtHubWithChassis extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Pose3d targetPose;
    private final PIDController rotationController;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private static final double kP = 3.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(2.0);

    public AimAtHubWithChassis(SwerveSubsystem swerveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        this.targetPose = AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D);

        // Initialize PID controller for rotation
        this.rotationController = new PIDController(kP, kI, kD);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        rotationController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerveSubsystem.getPose();

        double targetAngle = calculateTargetAngle(currentPose);

        double currentHeading = swerveSubsystem.getGyroHeading();

        double rotationSpeed = rotationController.calculate(currentHeading, targetAngle);

        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();

        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
    }

    /**
     * Calculates the angle the robot should face to aim at the target.
     * Simple calculation based on current position only.
     */

    private double calculateTargetAngle(Pose2d currentPose) {
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        return Math.atan2(dy, dx);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted or cancelled
        // You could optionally end when on target: return rotationController.atSetpoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(Translation2d.kZero, 0.0, true);
    }

}