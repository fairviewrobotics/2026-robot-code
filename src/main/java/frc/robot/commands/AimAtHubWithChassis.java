package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.SwerveSubsystem;
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

    // PID constants - tune these for your robot
    private static final double kP = 3.0;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    // Tolerance for considering the robot "on target" (radians)
    private static final double ANGLE_TOLERANCE = Math.toRadians(2.0);

    /**
     * Creates a command that aims the robot at the hub while allowing manual translation control.
     *
     * @param swerveSubsystem The swerve drive subsystem
     * @param red True if aiming at red alliance hub, false for blue alliance
     * @param xSupplier Supplier for X axis input (forward/back), typically from joystick
     * @param ySupplier Supplier for Y axis input (left/right), typically from joystick
     */


    public AimAtHubWithChassis(SwerveSubsystem swerveSubsystem, boolean red, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;

        // Set target pose based on alliance color
        if (red) {
            this.targetPose = FieldConstants.RED_HUB_POSE3D;
        } else {
            this.targetPose = FieldConstants.BLUE_HUB_POSE3D;
        }

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
        // Get current robot pose
        Pose2d currentPose = swerveSubsystem.getPose();

        // Calculate the angle to the target (simple, no motion compensation)
        double targetAngle = calculateTargetAngle(currentPose);

        // Get current robot heading
        double currentHeading = swerveSubsystem.getGyroHeading();

        // Calculate rotation speed using PID
        double rotationSpeed = rotationController.calculate(currentHeading, targetAngle);

        // Get joystick inputs for translation
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();

        // Drive the robot - joystick controls translation, PID controls rotation
        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);
    }

    /**
     * Calculates the angle the robot should face to aim at the target.
     * Simple calculation based on current position only.
     */
    private double calculateTargetAngle(Pose2d currentPose) {
        // Calculate vector from current robot position to target
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        // Calculate angle to target
        double angleToTarget = Math.atan2(dy, dx);

        Logger.recordOutput("AimSwerve/TargetPose", targetPose);

        return angleToTarget;
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted or cancelled
        // You could optionally end when on target: return rotationController.atSetpoint();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerveSubsystem.drive(Translation2d.kZero, 0.0, true);
    }

    /**
     * Returns true if the robot is currently aimed at the target within tolerance.
     */
    public boolean isOnTarget() {
        return rotationController.atSetpoint();
    }
}