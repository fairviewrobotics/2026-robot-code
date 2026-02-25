package frc.robot.utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record ShotData(
        double timestamp,
        double rawDistance,
        double virtualDistance,
        double turretAngle,
        double hoodAngle,
        double flywheelRPM,
        double timeOfFlight,
        ChassisSpeeds robotVelocity
) {}