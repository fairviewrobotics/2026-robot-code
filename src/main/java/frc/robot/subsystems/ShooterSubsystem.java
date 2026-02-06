package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShootingConstants;
import frc.robot.utils.TunableNumber;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX leftShooterMotor = new TalonFX(ShootingConstants.LEFT_SHOOTER_MOTOR_ID);
    private final TalonFX rightShooterMotor = new TalonFX(ShootingConstants.RIGHT_SHOOTER_MOTOR_ID);

    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    public ShooterSubsystem() {
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        updateHardwareConfigs();

        leftShooterMotor.getConfigurator().apply(motorConfig);
        rightShooterMotor.getConfigurator().apply(motorConfig);
    }

    private void updateHardwareConfigs() {
        var slot0 = motorConfig.Slot0;
        slot0.kP = ShootingConstants.SHOOTER_P.get();
        slot0.kD = ShootingConstants.SHOOTER_D.get();
        slot0.kS = ShootingConstants.SHOOTER_KS.get();
        slot0.kV = ShootingConstants.SHOOTER_KV.get();

        leftShooterMotor.getConfigurator().apply(slot0);
        rightShooterMotor.getConfigurator().apply(slot0);
    }

    public void setMotorRPM(double rpm) {
        double rps = rpm / 60.0;
        leftShooterMotor.setControl(new VelocityVoltage(rps));
        rightShooterMotor.setControl(new VelocityVoltage(rps));
    }

    @Override
    public void periodic() {
        TunableNumber.ifChanged(
                hashCode(),
                this::updateHardwareConfigs,
                ShootingConstants.SHOOTER_KS,
                ShootingConstants.SHOOTER_KV,
                ShootingConstants.SHOOTER_P,
                ShootingConstants.SHOOTER_D
        );

        SmartDashboard.putNumber("Shooter/left motor rpm", leftShooterMotor.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("Shooter/right motor rpm", rightShooterMotor.getVelocity().getValueAsDouble() * 60);
    }

    public void stopMotors() {
        leftShooterMotor.stopMotor();
        rightShooterMotor.stopMotor();
    }
}