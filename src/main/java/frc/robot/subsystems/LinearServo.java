package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShootingConstants;

public class LinearServo extends Servo {
    private final double length;

    public LinearServo(int channel, int length, int speed) {
        super(channel);
        this.length = length;
        // (Max, High-Deadband, Center, Low-Deadband, Min)
        this.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    }

    /**
     * @param setpoint Normalized position from 0.0 to 1.0
     */

    public void setPosition(double setpoint) {
        super.set(MathUtil.clamp(setpoint, 0.1, 0.9));
    }

    @Override
    public double getPosition() {
        // super.get() returns the last commanded position (0.0 to 1.0)
        return super.get();
    }
}