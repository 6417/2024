package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class Gyro {

    public static Gyro instance;

    private Gyro(){
        AHRS gyro = new AHRS(SPI.Port.kMXP);
    }

    public static Gyro getInstance(){
        if (instance == null){
            instance = new Gyro();
        }
        return instance;
    }
}
