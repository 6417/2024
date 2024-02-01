package frc.robot.subsystems.vision_autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {

    
    public static Gyro instance;
    AHRS gyro;

    private Gyro(){
        gyro = new AHRS(SPI.Port.kMXP);
    }

    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    public double getRot(){
        return gyro.getAngle();
    }

    public void reset(){
        gyro.reset();
    }

    public void setAngle(double angle){
        gyro.setAngleAdjustment(angle);
    }

    public static Gyro getInstance(){
        if (instance == null){
            instance = new Gyro();
        }
        return instance;
    }
}
