package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class Gyro {

    public static Gyro instance;
    AHRS gyro;

    private Gyro(){
        AHRS gyro = new AHRS(SPI.Port.kMXP);
    }

    public Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    public static Gyro getInstance(){
        if (instance == null){
            instance = new Gyro();
        }
        return instance;
    }
}
