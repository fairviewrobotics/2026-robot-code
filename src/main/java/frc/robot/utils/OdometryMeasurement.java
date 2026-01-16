package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public record OdometryMeasurement(double timestamp, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {}
