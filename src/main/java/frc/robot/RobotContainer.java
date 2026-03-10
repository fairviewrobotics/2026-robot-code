// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.autonomous.routines.BlueTrenchLeft;
import frc.robot.commands.*;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.*;

import java.io.File;

import frc.robot.utils.AllianceFlipUtil;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  final CommandPS5Controller primary_controller = new CommandPS5Controller(0);
  final CommandXboxController secondary_controller = new CommandXboxController(1);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
          "swerve"));
  private final Vision vision;
  TurretSubsystem turretSubsystem = new TurretSubsystem();
  HoodSubsystem hoodSubsystem = new HoodSubsystem(swerveSubsystem);
  // BallDetection ballDetection = new BallDetection(new PhotonCamera("limelight ball cam"), drivebase);
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  SuperSecretMissileTech superSecretMissileTech = new SuperSecretMissileTech(swerveSubsystem);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                  () -> primary_controller.getLeftY() * -1,
                  () -> primary_controller.getLeftX() * -1)
          .withControllerRotationAxis(() -> primary_controller.getRightX() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1.0)
          .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(primary_controller::getRightX,
                  primary_controller::getRightY)
          .headingWhile(true);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
                  () -> -primary_controller.getLeftY(),
                  () -> -primary_controller.getLeftX())
          .withControllerRotationAxis(() -> primary_controller.getRawAxis(
                  2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
          .withControllerHeadingAxis(() ->
                          Math.sin(
                                  primary_controller.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2),
                  () ->
                          Math.cos(
                                  primary_controller.getRawAxis(
                                          2) *
                                          Math.PI) *
                                  (Math.PI *
                                          2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(
                  0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    Vision.init(swerveSubsystem.getSwerveDrive());
    this.vision = Vision.getInstance();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
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
    Command driveFieldOrientedDirectAngle      = swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    // Field Y axis, not driver POV
    Command driveFieldOrientedDirectAngleKeyboard      = swerveSubsystem.driveFieldOriented(driveDirectAngleKeyboard);

    // ***TEST BINDINGS***
    primary_controller.L2().whileTrue(new IntakeCommand(intakeSubsystem));
    // secondary_controller.b().whileTrue(new RunCommand(() -> indexerSubsystem.setHopperMotorVoltage(-12)));
    secondary_controller.x().whileTrue(new IndexerCommand(indexerSubsystem));
    // primary_controller.R2().whileTrue(new ShooterCommand(shooterSubsystem, 1000, 1000));
    secondary_controller.pov(0).onTrue(new HoodTestCommand(hoodSubsystem));
    secondary_controller.pov(180).onTrue(Commands.runOnce(() -> hoodSubsystem.setHood(0.50)));
    secondary_controller.pov(90).whileTrue(new RunCommand(() -> turretSubsystem.setVoltage(2)));
    secondary_controller.pov(270).whileTrue(new RunCommand(() -> turretSubsystem.setVoltage(-2)));
//    secondary_controller.leftBumper().whileTrue(new RetractIntakeCommand(intakeSubsystem, 2));
    secondary_controller.a().whileTrue(new AimAtHub3(hoodSubsystem, shooterSubsystem, turretSubsystem, swerveSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d()).getTranslation()));
    secondary_controller.rightStick().onTrue(Commands.runOnce(() -> turretSubsystem.zeroTurretEncoder()));

//    secondary_controller.b().whileFalse(new RunCommand(() -> indexerSubsystem.setHopperMotorVoltage(0)));
//    secondary_controller.x().whileFalse(new RunCommand(() -> indexerSubsystem.setKickerMotorVoltage(0)));
    secondary_controller.pov(90).whileFalse(Commands.runOnce(() -> turretSubsystem.setVoltage(0)));
    secondary_controller.pov(270).whileFalse(Commands.runOnce(() -> turretSubsystem.setVoltage(0)));

    // *** ACTUAL BINDINGS ***

    primary_controller.pov(0).whileTrue(swerveSubsystem.sysIdDriveMotorCommand());
    primary_controller.pov(90).whileTrue(swerveSubsystem.sysIdAngleMotorCommand());
//    primary_controller.L2().whileTrue(new IntakeCommand(intakeSubsystem));
//    primary_controller.L1().onTrue(new RunCommand(swerveSubsystem::lock));
    primary_controller.R1().whileTrue(new FireShooterWithAgitation(intakeSubsystem));
    primary_controller.R2().whileTrue(new IndexerCommand(indexerSubsystem));
    primary_controller.cross().onTrue((Commands.runOnce(swerveSubsystem::zeroGyro)));
    primary_controller.options().onTrue(Commands.runOnce(() -> swerveSubsystem.resetOdometry(AllianceFlipUtil.apply(FieldConstants.BLUE_TRENCH_LEFT))));
//
    secondary_controller.leftTrigger().whileTrue(
            new AimAtHubWithChassis(
                    swerveSubsystem,
                    () -> 4.42 * MathUtil.applyDeadband(primary_controller.getLeftX(), 0.2),
                    () -> 4.42 * MathUtil.applyDeadband(primary_controller.getLeftY(), 0.2)
            )
    );
    //    secondary_controller.rightTrigger().whileTrue(new AgainstHubCommand(hoodSubsystem, shooterSubsystem, turretSubsystem));
    secondary_controller.leftBumper().whileTrue(new RetractIntakeCommand(intakeSubsystem, 2.0));
    secondary_controller.rightBumper().whileTrue(new ShooterCommand(shooterSubsystem, 1000, 1000));
//    secondary_controller.leftStick().onTrue(new RunCommand(intakeSubsystem::zeroIntakeDeployEncoder));
//    secondary_controller.rightStick().onTrue(new RunCommand(turretSubsystem::zeroTurretEncoder));
////    secondary_controller.pov(0).whileTrue(new TurretTestCommand(swerveSubsystem, turretSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_HUB_POSE3D.toPose2d())));
////    secondary_controller.pov(90).whileTrue(new TurretTestCommand(swerveSubsystem, turretSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_PASS_LEFT_POSE)));
////    secondary_controller.pov(270).whileTrue(new TurretTestCommand(swerveSubsystem, turretSubsystem, AllianceFlipUtil.apply(FieldConstants.BLUE_PASS_RIGHT_POSE)));
//    secondary_controller.pov(90).onTrue(new HoodTestCommand(hoodSubsystem));
//    secondary_controller.pov(0).onTrue(new RunCommand(() -> hoodSubsystem.setHood(15)));
//    secondary_controller.x().whileTrue(new TrenchLeftCommand(hoodSubsystem, shooterSubsystem, turretSubsystem));
//    secondary_controller.b().whileTrue(new TrenchRightCommand(hoodSubsystem, shooterSubsystem, turretSubsystem));
//    secondary_controller.y().whileTrue(new CornerLeftCommand(hoodSubsystem, shooterSubsystem, turretSubsystem));
//    secondary_controller.a().whileTrue(new CornerRightCommand(hoodSubsystem, shooterSubsystem, turretSubsystem));


    if (RobotBase.isSimulation())
    {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
              Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
              new ProfiledPIDController(5,
                      0,
                      0,
                      new Constraints(5, 2)),
              new ProfiledPIDController(5,
                      0,
                      0,
                      new Constraints(Units.degreesToRadians(360),
                              Units.degreesToRadians(180))
              ));


//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }

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

  public void setMotorBrake(boolean brake)
  {
    swerveSubsystem.setMotorBrake(brake);
  }

}