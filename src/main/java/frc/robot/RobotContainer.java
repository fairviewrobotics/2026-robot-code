// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autonomous.SuperSecretMissileTech;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.*;

import java.io.File;
import java.util.Optional;

import frc.robot.utils.NetworkTablesUtils;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller primary_controller = new CommandPS5Controller(0);
  final CommandXboxController secondary_controller = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Vision vision;
  // BallDetection ballDetection = new BallDetection();
  // ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NetworkTablesUtils NTAuto = NetworkTablesUtils.getTable("Autonomous");

  public static SuperSecretMissileTech superSecretMissileTech;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    Vision.init(swerveSubsystem);
    this.vision = Vision.getInstance();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
//    DogLog.setOptions(new DogLogOptions()
//            .withNtPublish(false)
//            .withNtTunables(true)
//    );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    swerveSubsystem.setDefaultCommand(
            new DriveCommands(
                    swerveSubsystem,
                    () ->
                            -MathUtil.applyDeadband(
                                    primary_controller.getLeftY(),
                                    DriveConstants.CONTROLLER_DEADBAND.get())
                                    * DriveConstants.MAX_XY_SPEED_MPS.get(),
                    () ->
                            -MathUtil.applyDeadband(
                                    primary_controller.getLeftX(),
                                    DriveConstants.CONTROLLER_DEADBAND.get())
                                    * DriveConstants.MAX_XY_SPEED_MPS.get(),
                    () ->
                            -MathUtil.applyDeadband(
                                    primary_controller.getRightX(),
                                    DriveConstants.CONTROLLER_DEADBAND.get())
                                    * Math.toRadians(DriveConstants.MAX_THETA_SPEED_RAD_PS.get()),
                    true));

    primary_controller.options().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
//    primary_controller.pov(0).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
//    primary_controller.pov(90).whileTrue(swerveSubsystem.sysIdAngleMotorCommand());



//    secondary_controller.x().whileTrue(new IntakeCommand(intakeSubsystem, shooterSubsystem, -IntakeConstants.INTAKING_VOLTAGE));
//    secondary_controller.rightBumper().whileTrue(new ShooterCommand(shooterSubsystem, ShootingConstants.TOP_SHOOTER_RPM.get(), ShootingConstants.BOTTOM_SHOOTER_RPM.get()));
//    secondary_controller.a().whileTrue(new IntakeCommand(intakeSubsystem, shooterSubsystem, IntakeConstants.INTAKING_VOLTAGE));

    // primary_controller.L1().whileTrue(new DriveToPoint(drivebase, robotState.getPose(), ballDetection.getBallPose(), 0.25));

    primary_controller.cross().onTrue((Commands.runOnce(swerveSubsystem::resetGyro)));
    primary_controller.options().whileTrue(Commands.none());
    // primary_controller.back().whileTrue(Commands.none());
    primary_controller.R1().onTrue(Commands.none());

    }


  public Command getAimAtHubCommand() {
    // RobotState robotState = RobotState.getInstance();
    Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

    if (allianceOpt.isEmpty()) {
      return Commands.none(); // or some fallback command
    }

    DriverStation.Alliance alliance = allianceOpt.get();
    Pose2d targetPose = (alliance == DriverStation.Alliance.Red)
            ? FieldConstants.RED_HUB_CENTER_POINT
            : FieldConstants.BLUE_HUB_CENTER_POINT;

    // Rotation2d targetAngle = robotState.getPose().minus(targetPose).getRotation();
    // double distance = robotState.getPose().getTranslation().getDistance(targetPose.getTranslation());

    // distance : hood angle map 8" increments
    double hoodAngle;

    return new ParallelCommandGroup(
            // set hood, rotate to angle
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand()
  {
    return superSecretMissileTech.getSelected();
  }

//  public void setMotorBrake(boolean brake)
//  {
//    swerveSubsystem.setMotorBrake(brake);
//  }

}