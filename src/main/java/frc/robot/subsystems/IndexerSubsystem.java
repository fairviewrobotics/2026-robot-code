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

    public IndexerSubsystem() {

        SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        kickerMotorConfig
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false);

        SparkFlexConfig hopperMotorConfig = new SparkFlexConfig();
        hopperMotorConfig
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false);

        kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
    }

    private final PIDController kickerPID = new PIDController(
            Preferences.getDouble("Kicker/kP", IndexerConstants.KICKER_P),
            0.0,
            Preferences.getDouble("Kicker/kD", IndexerConstants.KICKER_D)
    );

    private void initializePreferences() {
        Preferences.initDouble("Kicker/KICKER_RPM", IndexerConstants.KICKER_RPM);
        Preferences.initDouble("Hopper/HOPPER_RPM", 0.0);
        Preferences.initDouble("Kicker/kP", IndexerConstants.KICKER_P);
        Preferences.initDouble("Kicker/kD", IndexerConstants.KICKER_D);
    }

    public void setHopperMotorRPM(double speed) {
        hopperMotor.set(speed);
    }

    public void setKickerMotorRPM(double speed) {
        kickerMotor.set(speed);
    }

    public void setHopperWithPreferences() {
        hopperMotor.set(Preferences.getDouble("Hopper/HOPPER_RPM", IndexerConstants.HOPPER_MOTOR_RPM));
    }

    public void setKickerWithPreferences() {
        double pidOutput = kickerPID.calculate(kickerMotor.getEncoder().getVelocity(), Preferences.getDouble("Kicker/KICKER_RPM", IndexerConstants.KICKER_RPM));
        kickerMotor.setVoltage(pidOutput);
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
        Logger.recordOutput("Kicker/KICKER_RPM", kickerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Hopper/HOPPER_RPM", hopperMotor.getEncoder().getVelocity());
        kickerPID.setP(Preferences.getDouble("Kicker/kP", IndexerConstants.KICKER_P));
        kickerPID.setD(Preferences.getDouble("Kicker/kD", IndexerConstants.KICKER_D));
    }

}
