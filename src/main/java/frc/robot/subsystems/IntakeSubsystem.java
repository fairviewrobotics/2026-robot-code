package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.INTAKE_DEPLOY_MOTOR_ID, SparkFlex.MotorType.kBrushless);
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, SparkFlex.MotorType.kBrushless);

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

    }

    public void setIntakeRollerMotor(double speed) {intakeRollerMotor.set(speed);}

    public void setIntakeRollerMotorVoltage(double voltage) {intakeRollerMotor.setVoltage(voltage);}

    public void setIntakeDeployMotor(double speed) {intakeDeployMotor.set(speed);}

    public void stopMotors() {
        intakeDeployMotor.stopMotor();
        intakeRollerMotor.stopMotor();
    }

    public void setIntakeDeployMotorVoltage(double voltage) {intakeDeployMotor.setVoltage(voltage);}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/intake deploy motor", intakeDeployMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Intake/intake roller", intakeRollerMotor.getEncoder().getVelocity() * 60);
    }
}