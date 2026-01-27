package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;

public record SwerveModuleConfig(
        int moduleNumber,
        int driveID,
        int angleID,
        Translation2d moduleOffset,
        double absoluteEncoderOffset,
        boolean invertDrive,
        boolean invertAngle)
        {}
