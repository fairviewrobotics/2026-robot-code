package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex kickerMotor = new SparkFlex(IndexerConstants.KICKER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex hopperMotor = new SparkFlex(IndexerConstants.HOPPER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final int HOPPER_MAX_CURRENT_AMPS = 60;
    private final int KICKER_MAX_CURRENT_AMPS = 60;
    public IndexerSubsystem() {

        SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
        kickerMotorConfig
                .smartCurrentLimit(KICKER_MAX_CURRENT_AMPS)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(false);

        SparkFlexConfig hopperMotorConfig = new SparkFlexConfig();
        hopperMotorConfig
                .smartCurrentLimit(HOPPER_MAX_CURRENT_AMPS)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .inverted(true);

        kickerMotor.configure(kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hopperMotor.configure(hopperMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        initializePreferences();
    }

    private final PIDController kickerPID = new PIDController(
            Preferences.getDouble("Kicker/kP", IndexerConstants.KICKER_P),
            0.0,
            Preferences.getDouble("Kicker/kD", IndexerConstants.KICKER_D)
    );

    private final SimpleMotorFeedforward kickerFF = new SimpleMotorFeedforward(
            Preferences.getDouble("Kicker/kS", 0.0),
            Preferences.getDouble("Kicker/kV", 0.0),
            0.0
    );

    private void initializePreferences() {
        Preferences.initDouble("Kicker/KICKER_RPM", IndexerConstants.KICKER_RPM);
        Preferences.initDouble("Hopper/HOPPER_RPM", 0.0);
        Preferences.initDouble("Kicker/kP", IndexerConstants.KICKER_P);
        Preferences.initDouble("Kicker/kD", IndexerConstants.KICKER_D);
        Preferences.initDouble("Kicker/kS", 0.0);
        Preferences.initDouble("Kicker/kV", 0.0);
    }

    public void setHopperMotorPercent(double percent) {
        hopperMotor.set(percent);
    }

    public void setKickerMotorPercent(double percent) {
        kickerMotor.set(percent);
    }

    public void setHopperWithPreferences() {
        double percentage = MathUtil.clamp(Preferences.getDouble("Hopper/HOPPER_RPM", IndexerConstants.MAX_RPM_VORTEX)/IndexerConstants.MAX_RPM_VORTEX, 0.0, 1.0);
        hopperMotor.set(percentage);
    }

    public void setKickerWithPreferences() {
        double velocity = kickerMotor.getEncoder().getVelocity();
        double RPM = Preferences.getDouble("Kicker/KICKER_RPM", IndexerConstants.KICKER_RPM);
        double pidOutputVoltage = kickerPID.calculate(velocity, RPM);
        double ffOutputVoltage = kickerFF.calculate(RPM);
        kickerMotor.setVoltage(pidOutputVoltage + ffOutputVoltage);
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
        kickerFF.setKs(Preferences.getDouble("Kicker/kS", 0.0));
        kickerFF.setKv(Preferences.getDouble("Kicker/kV", 0.0));
    }

}
