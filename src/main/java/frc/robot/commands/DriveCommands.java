// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class DriveCommands extends Command {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier thetaSupplier;
    private boolean fieldRelative;

    public DriveCommands(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier forwardSupplier,
            DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSuplier,
            boolean fieldRelative) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;
        this.fieldRelative = fieldRelative;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        Logger.recordOutput("x-mps", xSupplier.getAsDouble());
        Logger.recordOutput("y-mps", ySupplier.getAsDouble());
        Logger.recordOutput("angle-rad-ps", thetaSupplier.getAsDouble());

        swerveSubsystem.drive(
                new Translation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                thetaSupplier.getAsDouble(),
                fieldRelative,
                false);
    }

}