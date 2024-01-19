package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Tankdrive_poseestimator;

public class Tankdrive extends DriveBase{

  //TalonFX rightfront = new TalonFX(1);
  // TalonFX leftfront = new TalonFX(2);
  TalonFX rigthfront;
  TalonFX leftfront;

  
  private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
  private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

  private StatusSignal rotorpos_left;
  private StatusSignal rotorpos_rigth;
  private StatusSignal rotorv_left;
  private StatusSignal rotorv_rigth;

  public DifferentialDrive differentialDrive;

  public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.7);

  public final DriveCommand drive_command;

  public static Tankdrive instance;

  public Tankdrive() {
    rigthfront = new TalonFX(Constants.Testchassi.idRigthfront);
    leftfront = new TalonFX(Constants.Testchassi.idLeftfront);

    leftfront.setInverted(true);

    rigthfront.setControl(new Follower(Constants.Testchassi.idRigthback, false));
    leftfront.setControl(new Follower(Constants.Testchassi.idLeftback, false));


    rotorpos_left =  leftfront.getRotorPosition();
    rotorpos_rigth = rigthfront.getRotorPosition();

    rotorv_left = leftfront.getRotorVelocity();
    rotorv_rigth = rigthfront.getRotorVelocity();

    differentialDrive = new DifferentialDrive(leftfront, rigthfront);

    //this.setDefaultCommand(drive_command);
    //drive_command.scedule;

    drive_command = new DriveCommand(this);
    this.setDefaultCommand(drive_command);
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
      Double.parseDouble(rotorv_left.getValue().toString()) * 10 * Constants.Testchassi.Odometry.encoderToMetersConversion,
      Double.parseDouble(rotorpos_rigth.getValue().toString()) * -10 * Constants.Testchassi.Odometry.encoderToMetersConversion);
  }

  public void setVolts(double leftvolts, double rigthvolts){
    leftfront.setVoltage(leftvolts);
    rigthfront.setVoltage(rigthvolts);
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

  //second method to drive robot but does not work
  public void drive2(double v_x, double v_y, double rot){
    m_rightOut.Output = step(v_x) * (v_x*v_x + rot)/2;
    m_leftOut.Output = step(v_x) * (v_x*v_x - rot)/2;
    System.out.println((v_x*v_x - rot)/2);
    System.out.println(v_x*v_y + rot);

    rigthfront.setControl(m_rightOut);
    leftfront.setControl(m_leftOut);
  }

  public void drive(double v_x, double v_y, double rot){
    //leftback.set(v_x);
    //rightback.set(v_x);
     differentialDrive.arcadeDrive(v_x, rot);
  }

  public static Tankdrive getInstance(){
      if (instance == null){
        instance = new Tankdrive();
      }
      return instance;
  }
}
