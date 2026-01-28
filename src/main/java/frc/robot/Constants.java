// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.TunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final boolean TUNING_MODE = true;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final double MAX_ANGULAR_SPEED = 0.5;
  public static final double BUFFER_SIZE_SECONDS = 2.0;

  public static TunableNumber TARGET_POSE_X = new TunableNumber("target_pose_x", 6.0);
  public static TunableNumber TARGET_POSE_Y = new TunableNumber("target_pose_y", 7.0);
  public static TunableNumber TARGET_POSE_ROTATION = new TunableNumber("target_pose_rotation", 0.0);

  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static TunableNumber DECELERATION_P = new TunableNumber("drive_controller_p",6.0);
    public static TunableNumber DECELERATION_D = new TunableNumber("drive_controller_d", 0.0);
    public static TunableNumber AUTO_ROTATION_P = new TunableNumber("rotation_controller_p", 6); //4?
    public static TunableNumber AUTO_ROTATION_D = new TunableNumber("rotation_controller_d", 0.0);
    public static final TrapezoidProfile.Constraints TRANSLATION_ALIGN_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_SPEED, 2.0);
    public static final TrapezoidProfile.Constraints ROTATION_ALIGN_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, 0.25);
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
