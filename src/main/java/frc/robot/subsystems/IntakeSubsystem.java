package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private SparkFlexConfig intakeDeployMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();

    // could be a boolean but might want to add more states later
    public enum IntakeState {
        DEPLOYED,
        RETRACTED
    }

    private IntakeState intakeState = IntakeState.RETRACTED;

//    private double errorThreshold = 0.5;
    // TODO: set proper velocity threshold for intake deployment
    private double velocityThreshold = 0.1;

//    double lastKP = IntakeConstants.INTAKE_DEPLOY_P;
//    double lastKD = IntakeConstants.INTAKE_DEPLOY_D;
    double lastRollerRPM = IntakeConstants.INTAKE_TUNED_RPM;
    double lastIntakeDeployVoltage = IntakeConstants.INTAKE_DEPLOY_MOTOR_VOLTAGE;
//    double lastDeploySetpoint = 0.0;
//    double lastDeployCurrent = IntakeConstants.DEPLOYED_CURRENT_LIMIT;
    private final int INTAKE_DEPLOY_MAX_CURRENT_AMPS = 40;
    private final int INTAKE_ROLLER_MAX_CURRENT_AMPS = 80;

    public IntakeSubsystem() {

        intakeDeployMotorConfig
                .smartCurrentLimit(INTAKE_DEPLOY_MAX_CURRENT_AMPS)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(true);

        intakeRollerMotorConfig
                .smartCurrentLimit(INTAKE_ROLLER_MAX_CURRENT_AMPS)
                .inverted(false);

        intakeRollerMotor.configure(intakeRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeDeployMotor.configure(intakeDeployMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
        updateCache();
    }
//
//    private final PIDController intakePID = new PIDController(
//            Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P),
//            0.0,
//            Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D)
//    );

    private void initializePreferences() {
        Preferences.initDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKE_TUNED_RPM);
        Preferences.initDouble("Intake/INTAKE_DEPLOY_MOTOR_VOLTAGE", IntakeConstants.INTAKE_DEPLOY_MOTOR_VOLTAGE);
//        Preferences.initDouble("Intake/DEPLOY_SETPOINT", 0.0);
//        Preferences.initDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P);
//        Preferences.initDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D);
//        Preferences.initDouble("Intake/VelocityThreshold", 0.5);
//        Preferences.initDouble("Intake/ErrorThreshold", 1.0);
//        Preferences.initInt("Intake/DeployCurrentLimit", IntakeConstants.DEPLOYED_CURRENT_LIMIT);
    }

    private void updateCache() {
//        lastKP = Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P);
//        lastKD = Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D);
//        lastDeploySetpoint = Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0);
        lastRollerRPM = Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKE_TUNED_RPM);
    }

    private boolean preferencesChanged() {
        return // lastKP != Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P)
//                || lastKD != Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D)
//                || lastDeploySetpoint != Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0)
                lastIntakeDeployVoltage != Preferences.getDouble("Intake/INTAKE_DEPLOY_MOTOR_VOLTAGE", IntakeConstants.INTAKE_DEPLOY_MOTOR_VOLTAGE) ||
                lastRollerRPM != Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKE_TUNED_RPM);
//                || lastDeployCurrent != Preferences.getDouble("Intake/DEPLOY_CURRENT_LIMIT", 0);
    }

//    public void setDeployCurrentLimit(int amps) {
//        intakeDeployMotorConfig.smartCurrentLimit(amps);
//        intakeDeployMotor.configure(intakeDeployMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
//    }

    public void setIntakeRollerMotor(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setIntakeRollerMotorVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    // Intake State enum logic
    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    // Swaps intake state when at velocity threshold for both deploy and retraction collision detection
    public void swapIntakeState() {
        double currentVelocity = intakeDeployMotor.getEncoder().getVelocity();

        if (velocityThreshold > currentVelocity) {
            if (intakeState == IntakeState.RETRACTED) {
                intakeState = IntakeState.DEPLOYED;
            }  else {
                intakeState = IntakeState.RETRACTED;
            }
        }
    }

    //    public void setIntakeDeployMotor(double speed) {
//        intakeDeployMotor.set(speed);
//    }

//    public void deployIntakeWithVoltage(double voltage){
//        if (intakeDeployMotor.getEncoder().getVelocity() < 1.0){
//            intakeDeployMotor.setVoltage(0.0);
//        }
//        else{
//            intakeDeployMotor.setVoltage(voltage);
//        }
//    }

//    /**
//     * Deploy the intake based on linear distance with collision detection
//     * @param position linear distance (inches)
//     */
//
//    public void deployIntakeToPosition(double position) {
//        double currentPosition = intakeDeployMotor.getEncoder().getPosition();
//        double error = position - currentPosition;
//
//        boolean atGoal = Math.abs(error) < errorThreshold;
//
//        if (!atGoal) {
//            setDeployCurrentLimit(40);
//            double pidOutput = intakePID.calculate(currentPosition, position);
//            intakeDeployMotor.set(pidOutput);
//        } else {
//            // HOLDING STATE: Become "Squishy"
//            setDeployCurrentLimit(Preferences.getInt("Intake/DeployCurrentLimit", IntakeConstants.DEPLOYED_CURRENT_LIMIT));
//            intakeDeployMotor.setVoltage(0.5);
//        }
//    }

//    public void deployIntakeToPositionWithPreferences() {
//        deployIntakeToPosition(Preferences.getDouble("Intake/DEPLOY_SETPOINT", 0.0));
//    }

    public void setIntakeRollerMotorWithPreferences() {
        double percent = MathUtil.clamp(Preferences.getDouble("Intake/ROLLER_RPM", IntakeConstants.INTAKE_TUNED_RPM)/IntakeConstants.MAX_RPM_VORTEX, 0.0, 1.0);
        setIntakeRollerMotor(percent);
    }

//    public void zeroIntakeDeployEncoder(){
//        intakeDeployMotor.getEncoder().setPosition(0);
//    }

    public void stopMotors() {
        intakeDeployMotor.stopMotor();
        intakeRollerMotor.stopMotor();
    }

    public double getDeployMotorPosition() {
        return intakeDeployMotor.getEncoder().getPosition();
    }

    public void setIntakeDeployMotorVoltage(double voltage) {
        intakeDeployMotor.setVoltage(voltage);
    }

//    private void updateValues() {
//        intakePID.setP(Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P));
//        intakePID.setD(Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D));
//    }

    @Override
    public void periodic() {

        if (preferencesChanged()) {
//            updateValues();
        }

        Logger.recordOutput("Intake/Position", intakeDeployMotor.getEncoder().getPosition());
        Logger.recordOutput("Intake/Velocity", intakeDeployMotor.getEncoder().getVelocity());
        Logger.recordOutput("Intake/Current", intakeDeployMotor.getOutputCurrent());
    }
}