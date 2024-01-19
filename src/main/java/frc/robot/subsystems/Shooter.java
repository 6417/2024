package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{

    TalonFX shooter = new TalonFX(Constants.Global.idShooterMotor);
    private final DutyCycleOut m_shooter = new DutyCycleOut(0);

    private static Shooter instance;

    private Shooter(){

    }

    public void shoot(){
        m_shooter.Output = 1;
        shooter.setControl(m_shooter);
    }

    public void stop_shoot(){
        m_shooter.Output = 0;
        shooter.setControl(m_shooter);
    }

    public static Shooter getInstance(){
        if (instance == null){
            instance = new Shooter();
        }
        return instance;
    }
}
