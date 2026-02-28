package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeJiggleCommand extends Command {

    // ---- TUNE THESE ---- //
    private static final double AGITATE_AMPLITUDE   = 2.0;  // How far to move in each direction (encoder units / inches)
    private static final int    AGITATE_OSCILLATIONS = 3;   // How many back-and-forth cycles to complete
    private static final double AT_GOAL_THRESHOLD   = 0.3;  // How close is "close enough" to switch direction (encoder units)

    private final IntakeSubsystem intakeSubsystem;
    private final double m_centerPosition; // Position to oscillate around

    private double m_targetPosition;
    private int m_stepsCompleted; // Each half-swing counts as one step
    private final int m_totalSteps; // 2 steps per oscillation (out + back)

    /**
     * Agitates the intake around its current position when the command is scheduled.
     */
    public IntakeJiggleCommand(IntakeSubsystem intake) {
        this.intakeSubsystem = intake;
        m_centerPosition = 0; // Will be set in initialize()
        m_totalSteps = AGITATE_OSCILLATIONS * 2;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_stepsCompleted = 0;
        // Start by moving forward (center + amplitude)
        double startPos = intakeSubsystem.getDeployMotorPosition();
        m_targetPosition = startPos + AGITATE_AMPLITUDE;
    }

    @Override
    public void execute() {
        intakeSubsystem.deployIntakeToPosition(m_targetPosition);

        double currentPosition = intakeSubsystem.getDeployMotorPosition();
        boolean atTarget = Math.abs(currentPosition - m_targetPosition) < AT_GOAL_THRESHOLD;

        if (atTarget) {
            m_stepsCompleted++;
            if (m_stepsCompleted % 2 == 1) {
                m_targetPosition -= AGITATE_AMPLITUDE * 2; // Swing back
            } else {
                m_targetPosition += AGITATE_AMPLITUDE * 2; // Swing forward
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Return to center / stop
        intakeSubsystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return m_stepsCompleted >= m_totalSteps;
    }
}