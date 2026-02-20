package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private SparkFlexConfig intakeDeployMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();

    private double errorThreshold = 1.0; // inches - only check if we're this far from target
    private static final int STALL_SAMPLES_REQUIRED = 10; // ~200ms at 20ms loop

    double lastKP = IntakeConstants.INTAKE_DEPLOY_P;
    double lastKD = IntakeConstants.INTAKE_DEPLOY_D;
    double lastRollerRPM = IntakeConstants.INTAKING_RPM;
    double lastDeploySetpoint = 0.0;

    public IntakeSubsystem() {


        intakeDeployMotorConfig
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false)
                .encoder.positionConversionFactor(IntakeConstants.INTAKE_DEPLOY_MOTOR_CONVERSION_FACTOR)
                .velocityConversionFactor(IntakeConstants.INTAKE_DEPLOY_MOTOR_CONVERSION_FACTOR / 60.0); // Convert RPM to units/sec

        intakeRollerMotorConfig
                .inverted(false);

        intakeRollerMotor.configure(intakeRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeDeployMotor.configure(intakeDeployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
        updateCache();
    }

    private final PIDController intakePID = new PIDController(
            Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P),
            0.0,
            Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D)
    );

    private void initializePreferences() {
        Preferences.initDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKING_RPM);
        Preferences.initDouble("Intake/DEPLOY_SETPOINT", 0.0);
        Preferences.initDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P);
        Preferences.initDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D);
        Preferences.initDouble("Intake/VelocityThreshold", 0.5);
        Preferences.initDouble("Intake/ErrorThreshold", 1.0);
        Preferences.initInt("Intake/DeployCurrentLimit", IntakeConstants.DEPLOYED_CURRENT_LIMIT);
    }

    private void updateCache() {
        lastKP = Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P);
        lastKD = Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D);
        lastDeploySetpoint = Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0);
        lastRollerRPM = Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKING_RPM);
    }

    private boolean preferencesChanged() {
        return lastKP != Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P)
                || lastKD != Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D)
                || lastDeploySetpoint != Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0)
                || lastRollerRPM != Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKING_RPM);
    }

    public void setDeployCurrentLimit(int amps) {
        intakeDeployMotorConfig.smartCurrentLimit(amps);
        intakeDeployMotor.configure(intakeDeployMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIntakeRollerMotor(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setIntakeRollerMotorVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    public void setIntakeDeployMotor(double speed) {
        intakeDeployMotor.set(speed);
    }

    /**
     * Deploy the intake based on linear distance with collision detection
     * @param position linear distance (inches)
     */

    public void deployIntakeToPosition(double position) {
        double currentPosition = intakeDeployMotor.getEncoder().getPosition();
        double error = position - currentPosition;

        // 1. Determine if we are "At Goal"
        boolean atGoal = Math.abs(error) < errorThreshold;

        if (!atGoal) {
            // MOVING STATE: High strength to get there
            setDeployCurrentLimit(40);
            double pidOutput = intakePID.calculate(currentPosition, position);
            intakeDeployMotor.set(pidOutput);
        } else {
            // HOLDING STATE: Become "Squishy"
            // Drop current limit so it can be back-driven by a hit
            setDeployCurrentLimit(Preferences.getInt("Intake/DeployCurrentLimit", IntakeConstants.DEPLOYED_CURRENT_LIMIT));
            intakeDeployMotor.setVoltage(0.5);
        }
    }

    public void deployIntakeToPositionWithPreferences() {
        deployIntakeToPosition(Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0));
    }

    public void setIntakeRollerMotorWithPreferences() {
        setIntakeRollerMotor(Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKING_RPM));
    }

    public void stopMotors() {
        intakeDeployMotor.stopMotor();
        intakeRollerMotor.stopMotor();
    }

    public void setIntakeDeployMotorVoltage(double voltage) {
        intakeDeployMotor.setVoltage(voltage);
    }

    private void updateValues() {
        intakePID.setP(Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P));
        intakePID.setD(Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D));
    }

    @Override
    public void periodic() {
        if (preferencesChanged()) {
            updateValues();
        }

        Logger.recordOutput("Intake/Position", intakeDeployMotor.getEncoder().getPosition());
        Logger.recordOutput("Intake/Velocity", intakeDeployMotor.getEncoder().getVelocity());
        Logger.recordOutput("Intake/Current", intakeDeployMotor.getOutputCurrent());
    }
}