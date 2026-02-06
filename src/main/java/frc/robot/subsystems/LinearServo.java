package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShootingConstants;

public class LinearServo extends Servo{
    double speed;
    double length;
    double setPosition;
    double curPos;

    public LinearServo(int channel, int length, int speed) {
        super(channel);
        this.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
        this.length = length;
        this.speed = speed;
    }

    public void setPosition(double setpoint){
        setPosition = MathUtil.clamp(setpoint, 0, length);
        setSpeed( (setPosition/length *2)-1);
    }


    double lastTime = 0;

    public void updateCurPos(){
        double delta = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPosition + speed * delta){
            curPos -= speed * delta;
        } else if(curPos < setPosition - speed * delta){
            curPos += speed * delta;
        }else{
            curPos = setPosition;
        }
    }

    public double getPosition(){
        return curPos;
    }

    public boolean isFinished(){
        return curPos == setPosition;
    }

}