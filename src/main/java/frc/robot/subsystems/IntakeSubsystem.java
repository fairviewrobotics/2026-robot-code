package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkFlex intakeRollerMotor = new SparkFlex(IntakeConstants.INTAKE_ROLLER_MOTOR_ID, SparkFlex.MotorType.kBrushless);
    private final SparkFlex indexerMotor = new SparkFlex(IntakeConstants.INDEXER_ROLLER_MOTOR_ID, SparkFlex.MotorType.kBrushless);

    private final DigitalInput intakeLinebreak = new DigitalInput(0);

    public IntakeSubsystem() {
        SparkFlexConfig intakeRollerMotorConfig = new SparkFlexConfig();
        SparkFlexConfig indexerMotorConfig = new SparkFlexConfig();

        intakeRollerMotorConfig
                .inverted(true);
        indexerMotorConfig
                .inverted(false);

    }

    public void setSpeed(double speed) {
        intakeRollerMotor.set(speed);
    }

    public void setVoltage(double voltage) {
        intakeRollerMotor.setVoltage(voltage);
    }

    public void setIndexerVoltage(double voltage) {
        indexerMotor.setVoltage(voltage);
    }

    public boolean getLinebreak() {
        return !intakeLinebreak.get();
    }

    @Override
    public void periodic() {

    }
}
