package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Tankdrive_poseestimator;

public class Tankdrive extends DriveBase{

  //TalonFX rightfront = new TalonFX(1);
  // TalonFX leftfront = new TalonFX(2);
  TalonFX rightback;
  TalonFX leftback;

  
  private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
  private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

  private StatusSignal rotorpos_left = leftback.getRotorPosition();
  private StatusSignal rotorpos_rigth = rightback.getRotorPosition();
  private StatusSignal rotorv_left = leftback.getRotorVelocity();
  private StatusSignal rotorv_rigth = rightback.getRotorVelocity();

  public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.7);

  public final DriveCommand drive_command;

  public static Tankdrive instance;

  public Tankdrive() {
    drive_command = new DriveCommand(this);

    rightback = new TalonFX(Constants.Testchassi.idRigthback);
    leftback = new TalonFX(Constants.Testchassi.idLeftback);

    rightback.setControl(new Follower(Constants.Testchassi.idRigthfront, true));
    leftback.setControl(new Follower(Constants.Testchassi.idLeftfront, false));

    //this.setDefaultCommand(drive_command);
    //drive_command.scedule();
  }

  @Override
  public void periodic() {}

  public double getLeftEndcoderPos(){
    rotorpos_left.refresh();
    return Double.parseDouble(rotorpos_left.getValue().toString());
  }

  public double getRigthEndcoderPos(){
    rotorpos_rigth.refresh();
    return Double.parseDouble(rotorpos_rigth.getValue().toString());
  }

  public DifferentialDriveWheelSpeeds getWeelSpeeds(){
    rotorv_left.refresh();
    rotorv_rigth.refresh();
    return new DifferentialDriveWheelSpeeds(
      Double.parseDouble(rotorv_left.getValue().toString()) * 10 * Constants.TalonFX_endcoders_to_meters,
      Double.parseDouble(rotorpos_rigth.getValue().toString()) * -10 * Constants.TalonFX_endcoders_to_meters);
  }

  public void setVolts(double leftvolts, double rigthvolts){
    leftback.setVoltage(leftvolts);
    rightback.setVoltage(rigthvolts);
    //m_drive.feed();
  }
  
  public Pose2d getPos(){
    return Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition();
  }

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
