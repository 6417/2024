package frc.robot.subsystems.drive;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class Tankdrive extends DriveBase{

  //TalonFX rightfront = new TalonFX(1);
  // TalonFX leftfront = new TalonFX(2);
  TalonFX rightback = new TalonFX(Constants.Testchassi.idRigthback);
  TalonFX leftback = new TalonFX(Constants.Testchassi.idLeftback);

  
  private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
  private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

  public final DriveCommand drive_command;

  public static Tankdrive instance;

  public Tankdrive() {
    drive_command = new DriveCommand(this);
    

    rightback.setControl(new Follower(Constants.Testchassi.idRigthfront, true));
    leftback.setControl(new Follower(Constants.Testchassi.idLeftback, false));

    this.setDefaultCommand(drive_command);
  }

  @Override
  public void periodic() {}

  private double step(double number){
    if (number >= 0){
      return 1;
    }
    else{
      return -1;
    }
  }

  public void drive(double v_x, double v_y, double rot){
    m_rightOut.Output = step(v_x) * (v_x*v_x + rot)/2;
    m_leftOut.Output = step(v_x) * (v_x*v_x - rot)/2;
    System.out.println((v_x*v_x - rot)/2);

    rightback.setControl(m_rightOut);
    leftback.setControl(m_leftOut);
  }

  public static Tankdrive getInstance(){
      if (instance == null){
        instance = new Tankdrive();
      }
      return instance;
  }
}
