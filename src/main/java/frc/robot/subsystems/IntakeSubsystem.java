package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShootingConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkFlex.MotorType.kBrushless);
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, SparkFlex.MotorType.kBrushless);

    double lastKP = IntakeConstants.INTAKE_DEPLOY_P;
    double lastKD = IntakeConstants.INTAKE_DEPLOY_D;
    double lastRollerRPM = IntakeConstants.INTAKING_RPM;
    double lastDeploySetpoint = 0.0;

    public IntakeSubsystem() {

        SparkFlexConfig intakeDeployMotorConfig = new SparkFlexConfig();
        SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();

        intakeDeployMotorConfig
                .inverted(true);
        intakeDeployMotorConfig.encoder
                .positionConversionFactor(IntakeConstants.INTAKE_DEPLOY_MOTOR_CONVERSION_FACTOR)
                .inverted(false);

        intakeRollerMotorConfig
                .inverted(false);

        intakeRollerMotor.configure(intakeRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeDeployMotor.configure(intakeDeployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
        updateCache();

    }

    private final PIDController intakePID = new PIDController(
            Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P), 0.0, Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_D)
    );

    private void initializePreferences() {
        Preferences.initDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKING_RPM);
        Preferences.initDouble("Intake/DEPLOY_SETPOINT", 0.0);
        Preferences.initDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P);
        Preferences.initDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D);
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

    public void setIntakeRollerMotor(double speed) {intakeRollerMotor.set(speed);}

    public void setIntakeRollerMotorVoltage(double voltage) {intakeRollerMotor.setVoltage(voltage);}

    public void setIntakeDeployMotor(double speed) {intakeDeployMotor.set(speed);}

    /**
     * Deploy the intake based on linear distance
     * @param position linear distance (m)
     */

    public void deployIntakeToPosition(double position) {
        intakeDeployMotor.set(intakePID.calculate(intakeDeployMotor.getEncoder().getPosition(), position));
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

    public void setIntakeDeployMotorVoltage(double voltage) {intakeDeployMotor.setVoltage(voltage);}

    private void updateValues() {
        intakePID.setP(Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P));
        intakePID.setD(Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D));
    }

    @Override
    public void periodic() {
        if (preferencesChanged()) {
            updateValues();
        }
    }

}