package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftShooterMotor = new TalonFX(ShootingConstants.LEFT_SHOOTER_MOTOR_ID);
    private final TalonFX rightShooterMotor = new TalonFX(ShootingConstants.RIGHT_SHOOTER_MOTOR_ID);

    private double lastKP = ShootingConstants.DEFAULT_KP;
    private double lastKI = ShootingConstants.DEFAULT_KI;
    private double lastKD = ShootingConstants.DEFAULT_KD;
    private double lastKV = ShootingConstants.DEFAULT_KV;
    private double lastKS = ShootingConstants.DEFAULT_KS;
    private double lastLRPM = ShootingConstants.LEFT_SHOOTER_RPM;
    private double lastRRPM = ShootingConstants.RIGHT_SHOOTER_RPM;

    private final InterpolatingDoubleTreeMap DistanceToRPM =
            new InterpolatingDoubleTreeMap();

    private final InterpolatingDoubleTreeMap DistanceToShotTime =
            new InterpolatingDoubleTreeMap();

    private final BangBangController shooterBangController = new BangBangController();

    public ShooterSubsystem() {
        initializePreferences();

        TalonFXConfiguration leftShooterMotorConfig = new TalonFXConfiguration();
        leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftShooterMotorConfig.Slot0.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        leftShooterMotorConfig.Slot0.kI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        leftShooterMotorConfig.Slot0.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        leftShooterMotorConfig.Slot0.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        leftShooterMotorConfig.Slot0.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        leftShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        leftShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        leftShooterMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        leftShooterMotorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        TalonFXConfiguration rightShooterMotorConfig = new TalonFXConfiguration();
        rightShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rightShooterMotorConfig.Slot0.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        rightShooterMotorConfig.Slot0.kI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        rightShooterMotorConfig.Slot0.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        rightShooterMotorConfig.Slot0.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        rightShooterMotorConfig.Slot0.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        rightShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        rightShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        rightShooterMotorConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
        rightShooterMotorConfig.MotorOutput.PeakReverseDutyCycle = 0.0;

        leftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);
        rightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);

        updateCache();
        createDistanceToRPMMap();
        createDistanceToShotTimeMap();
    }

    private void initializePreferences() {
        Preferences.initDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        Preferences.initDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        Preferences.initDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        Preferences.initDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        Preferences.initDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);
        Preferences.initDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM);
        Preferences.initDouble("Shooter/RIGHT_RPM_SETPOINT", ShootingConstants.RIGHT_SHOOTER_RPM);
    }

    private void updateCache() {
        lastKP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        lastKI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        lastKD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        lastKV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        lastKS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);
        lastLRPM = Preferences.getDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM);
        lastRRPM = Preferences.getDouble("Shooter/RIGHT_RPM_SETPOINT", ShootingConstants.RIGHT_SHOOTER_RPM);
    }

    private boolean preferencesChanged() {
        return lastKP != Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP)
                || lastKI != Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI)
                || lastKD != Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD)
                || lastKV != Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV)
                || lastKS != Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS)
                || lastLRPM != Preferences.getDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM)
                || lastRRPM != Preferences.getDouble("Shooter/RIGHT_RPM_SETPOINT", ShootingConstants.RIGHT_SHOOTER_RPM);
    }

    private void updateHardwareConfigs() {
        var slot0Config = new TalonFXConfiguration().Slot0;
        slot0Config.kP = Preferences.getDouble("Shooter/kP", ShootingConstants.DEFAULT_KP);
        slot0Config.kI = Preferences.getDouble("Shooter/kI", ShootingConstants.DEFAULT_KI);
        slot0Config.kD = Preferences.getDouble("Shooter/kD", ShootingConstants.DEFAULT_KD);
        slot0Config.kV = Preferences.getDouble("Shooter/kV", ShootingConstants.DEFAULT_KV);
        slot0Config.kS = Preferences.getDouble("Shooter/kS", ShootingConstants.DEFAULT_KS);

        leftShooterMotor.getConfigurator().apply(slot0Config);
        rightShooterMotor.getConfigurator().apply(slot0Config);

        updateCache();
    }

    public void setMotorRPM(double rpm) {
        double rps = rpm / 60.0;
        leftShooterMotor.setControl(new VelocityVoltage(rps));
        rightShooterMotor.setControl(new VelocityVoltage(rps));
    }

    public void setMotorRPMBangBang(double rpm) {
        double bangOutput = shooterBangController.calculate(leftShooterMotor.getVelocity().getValueAsDouble() * 60, rpm);
        leftShooterMotor.setVoltage(bangOutput * 12.0);
        rightShooterMotor.setVoltage(bangOutput * 12.0);
    }

    public void setMotorRPMBangBangWithPreferences() {
        this.setMotorRPMBangBang(Preferences.getDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM));
    }

    @Override
    public void periodic() {

        if (preferencesChanged()) {
            updateHardwareConfigs();
        }

        Logger.recordOutput("Shooter/left motor rpm", leftShooterMotor.getVelocity().getValueAsDouble() * 60);
        Logger.recordOutput("Shooter/right motor rpm", rightShooterMotor.getVelocity().getValueAsDouble() * 60);

        Logger.recordOutput("Shooter/Current kP", lastKP);
        Logger.recordOutput("Shooter/Current kI", lastKI);
        Logger.recordOutput("Shooter/Current kD", lastKD);
        Logger.recordOutput("Shooter/Current kV", lastKV);
        Logger.recordOutput("Shooter/Current kS", lastKS);
    }

    public void stopMotors() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }

    public void setLeftShooterMotor(double rpm) {
        leftShooterMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public void setRightShooterMotor(double rpm) {
        rightShooterMotor.setControl(new VelocityVoltage(rpm/60));
    }

    public void setLeftShooterMotorVoltage(double voltage) {
        leftShooterMotor.setVoltage(voltage);
    }

    public void setBothMotorsPreferences() {
        setLeftShooterMotor(Preferences.getDouble("Shooter/LEFT_RPM_SETPOINT", ShootingConstants.LEFT_SHOOTER_RPM));
        setRightShooterMotor(Preferences.getDouble("Shooter/RIGHT_RPM_SETPOINT", ShootingConstants.RIGHT_SHOOTER_RPM));
    }

    private void createDistanceToRPMMap() {
        DistanceToRPM.put(0.0, 2000.0);
        DistanceToRPM.put(3.0796, 2350.0);
        DistanceToRPM.put(4.1596, 2600.0);
        DistanceToRPM.put(5.1396, 3200.0);
        DistanceToRPM.put(8.0, 4500.0);
    }

    private void createDistanceToShotTimeMap() {
        DistanceToShotTime.put(0.0, 0.3);
        DistanceToShotTime.put(3.0796, 0.785);
        DistanceToShotTime.put(4.1596, 0.995);
        DistanceToShotTime.put(5.1396, 1.265);
        DistanceToShotTime.put(8.0, 2.0);
    }

    public double getDistanceToRPMMap(double distance) {return DistanceToRPM.get(distance);}
    public double getDistanceToShotTime(double distance) {return DistanceToShotTime.get(distance);}

}