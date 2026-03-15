package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
 * Command to aim the rear of the robot (where the offset turret is) at the hub
 * while allowing manual joystick control for translation.
 */
public class AimAtHubWithChassis extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final PIDController rotationController;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    // Tuning constants - might need adjustment based on robot weight/traction
    private static final double kP = 0.5;
    private static final double kI = 0.0;
    private static final double kD = 0.1;

    private static final double ANGLE_TOLERANCE = Units.degreesToRadians(0.5);

    public AimAtHubWithChassis(SwerveSubsystem swerveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationController = new PIDController(kP, kI, kD);
        // Standard WPILib rotation is -PI to PI
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
        // 1. Get positions
        Pose2d targetPose = AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d());
        Pose2d currentPose = swerveSubsystem.getPose();

        // 2. Calculate the field-relative angle from robot center to hub
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double angleToHub = Math.atan2(dy, dx);

        // 3. Flip the heading!
        // Since your turret is on the back, the front of the chassis should point
        // 180 degrees (PI radians) AWAY from the hub.
        double desiredRobotHeading = MathUtil.angleModulus(angleToHub + Math.PI);

        // 4. Calculate rotation speed
        double currentHeading = currentPose.getRotation().getRadians();
        double rotationSpeed = rotationController.calculate(currentHeading, desiredRobotHeading);

        // 5. Get translation speeds from suppliers
        double xSpeed = xSupplier.getAsDouble();
        double ySpeed = ySupplier.getAsDouble();

        // 6. Drive
        // 'true' for fieldRelative ensures translation doesn't spin with the robot
        swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rotationSpeed, true);

        // Logging for debugging
        Logger.recordOutput("Drive/ChassisAimTargetAngle", desiredRobotHeading);
    }

    @Override
    public boolean isFinished() {
        // This is a 'whileTrue' command, so it only ends when button is released
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotation when the button is released
        swerveSubsystem.drive(Translation2d.kZero, 0.0, true);
    }
}