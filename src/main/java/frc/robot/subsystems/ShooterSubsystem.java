package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.TunableNumber;
import frc.robot.utils.MathUtils;
import frc.robot.utils.NetworkTablesUtils;

import static frc.robot.Constants.TARGET_POSE_ROTATION;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex topShooterMotor = new SparkFlex(ShooterConstants.TOP_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final SparkFlex bottomShooterMotor = new SparkFlex(ShooterConstants.BOTTOM_SHOOTER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private final DigitalInput shooterLinebreak = new DigitalInput(10);
    NetworkTablesUtils shooterNT = NetworkTablesUtils.getTable("Shooter");

    public ShooterSubsystem() {

        SparkFlexConfig topShooterMotorConfig = new SparkFlexConfig();
        SparkFlexConfig bottomShooterMotorConfig = new SparkFlexConfig();

        topShooterMotorConfig
                .inverted(true);
        bottomShooterMotorConfig
                .inverted(true);

    }

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        ShooterConstants.SHOOTER_KS,
        ShooterConstants.SHOOTER_KV,
        ShooterConstants.SHOOTER_KA
    );

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D
    );

    double setpoint;

    public void setTopMotorRPM(double rpm) {
        this.setpoint = rpm;
        topShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

    public void setBottomMotorRPM(double rpm) {
        this.setpoint = rpm;
        bottomShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(bottomShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

    public void setMotorRPM(double rpm) {
        this.setpoint = rpm;
        topShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) + 
                shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );

        bottomShooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(bottomShooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) + 
                shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );

        // indexerMotor.set(rpm/2);
    }

    public void stopMotors() {
        topShooterMotor.setVoltage(0);
        bottomShooterMotor.setVoltage(0);
    }

    public void runVolts(double volts) {
        topShooterMotor.setVoltage(volts);
        bottomShooterMotor.setVoltage(volts);
    }

    public boolean getLinebreak() {
        return !shooterLinebreak.get();
    }


    public void periodic() {

        shooterNT.setEntry("shooter error", shooterPID.getError());
        shooterNT.setEntry("shooter setpoint", setpoint);
        shooterNT.setEntry("shooter velocity", MathUtils.RPMtoRadians(topShooterMotor.getEncoder().getVelocity()));

    }

    public void resetPID() {
        shooterPID.reset();
    }
}