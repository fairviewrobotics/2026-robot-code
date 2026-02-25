package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex kickerMotor = new SparkFlex(IndexerConstants.KICKER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex hopperMotor = new SparkFlex(IndexerConstants.HOPPER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

    private SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig hopperMotorConfig = new SparkFlexConfig();

    public IndexerSubsystem() {

        kickerMotorConfig
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false);

        hopperMotorConfig
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false);

        kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
    }

    private final PIDController intakePID = new PIDController(
            Preferences.getDouble("Intake/kP", IntakeConstants.INTAKE_DEPLOY_P),
            0.0,
            Preferences.getDouble("Intake/kD", IntakeConstants.INTAKE_DEPLOY_D)
    );

    private void initializePreferences() {
        Preferences.initDouble("Indexer/KICKER_RPM", IntakeConstants.INTAKING_RPM);
        Preferences.initDouble("Indexer/HOPPER_RPM", 0.0);
    }

    public void setHopperMotorRPM(double speed) {
        hopperMotor.set(speed);
    }

    public void setKickerMotorRPM(double speed) {
        kickerMotor.set(speed);
    }

    public void setHopperWithPreferences() {
        hopperMotor.set(Preferences.getDouble("Indexer/HOPPER_RPM", IndexerConstants.HOPPER_MOTOR_RPM));
    }

    public void setKickerWithPreferences() {
        kickerMotor.set(Preferences.getDouble("Indexer/KICKER_RPM", IndexerConstants.KICKER_RPM));
    }

    public void setKickerMotorVoltage(double voltage) {
        kickerMotor.setVoltage(voltage);
    }

    public void setHopperMotorVoltage(double voltage) {
        hopperMotor.setVoltage(voltage);
    }


    public void stopMotors() {
        kickerMotor.stopMotor();
        hopperMotor.stopMotor();
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/KICKER_RPM", kickerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Indexer/HOPPER_RPM", hopperMotor.getEncoder().getVelocity());
    }

}
