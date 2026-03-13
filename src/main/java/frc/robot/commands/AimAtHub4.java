package frc.robot.commands;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShootingConstants;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

//IMPORTANT x,y,z is forward/back,Left/Right,Up/Down
public class AimAtHub4 extends Command {
    TurretSubsystem turret;
    ShooterSubsystem shooter;
    HoodSubsystem hood;
    SwerveSubsystem swerve;
    Pose3d curretPose;
    Translation3d targetPose;
    double RPM;
    Transform3d turretOffset = ShootingConstants.TURRET_OFFSET3D;
    double velocity;
    double d;
    double t;
    Pose3d futerPose;
    Pose3d relativePose;
    boolean low;

    public AimAtHub4(TurretSubsystem turret, ShooterSubsystem shooter, HoodSubsystem hood, SwerveSubsystem swerve, Translation3d targetPose, double RPM, boolean low){
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;
        this.swerve = swerve;
        this.curretPose = new Pose3d(swerve.getPose().getX(), swerve.getPose().getY(), 0.0, new Rotation3d(swerve.getPose().getRotation()));
        curretPose.transformBy(ShootingConstants.TURRET_OFFSET3D);
        this.targetPose = targetPose;
        this.RPM = RPM;
        this.velocity = RPM * 3 * 0.001329;
        this.d = Math.hypot(targetPose.getX()-curretPose.getX(), targetPose.getY()-curretPose.getY());
        this.t = this.d/this.velocity;
        this.futerPose = futerPose(3);
        this.relativePose = new Pose3d(targetPose.getX()-futerPose.getX(),targetPose.getY()-futerPose.getY(),targetPose.getZ()-futerPose.getZ(),futerPose.getRotation());
        this.low = low;
    }

    private Pose3d futerPose(int n){
        Pose3d futerPose =curretPose.exp(new Twist3d(swerve.getFieldVelocity().vxMetersPerSecond * t,
                    swerve.getFieldVelocity().vyMetersPerSecond * t,
                    0.0,
                    swerve.getFieldVelocity().omegaRadiansPerSecond * t,
                    0.0,
                    0.0));
        for(int i = 0; i< n; i++){
            d = Math.hypot(targetPose.getX()-futerPose.getX(), targetPose.getY()-futerPose.getY());
            t = d/velocity;
            futerPose = curretPose.exp(new Twist3d(swerve.getFieldVelocity().vxMetersPerSecond * t,
                    swerve.getFieldVelocity().vyMetersPerSecond * t,
                    0.0,
                    swerve.getFieldVelocity().omegaRadiansPerSecond * t,
                    0.0,
                    0.0));
        }
        return futerPose;

    }

    public double HoodAngle(){
        double dx = relativePose.getX();
        double dy = relativePose.getY();
        double dz = relativePose.getZ();
        double d = Math.hypot(dx, dy);
        double g = 9.8;
        double v2 = velocity * velocity;
        double discriminant = v2 * v2 - g * (g * Math.pow(d, 2) + 2 * dz * v2);
        if (discriminant < 0) return -1; // unreachable
        double angleLow = Math.atan((v2 - Math.sqrt(discriminant)) / (g * d));
        double angleHigh =  Math.atan((v2 + Math.sqrt(discriminant)) / (g * d));
        if(low) {
            return angleLow;
        }
        return angleHigh;
    }

    public double TurretAngle(){
        double turretAngle = Math.atan2(relativePose.getY(),relativePose.getX());
        if(turretAngle < 0){
            turretAngle = Math.PI * 2 + turretAngle;
        }
        return turretAngle;
    }

    @Override
    public void execute(){
        shooter.setMotorRPM(RPM);
        turret.setTurret(Math.toDegrees(TurretAngle() + swerve.getPose().getRotation().getRadians()));
        hood.setHood(HoodAngle());
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
        turret.setVoltage(0.0);
    }

}
