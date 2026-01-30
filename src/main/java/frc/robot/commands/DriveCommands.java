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

    private SwerveSubsystem swerveSubsystem;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier thetaSupplier;
    private boolean fieldRelative;

    public DriveCommands(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier thetaSupplier,
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