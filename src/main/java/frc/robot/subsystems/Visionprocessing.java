package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionprocessing extends SubsystemBase {

    static Visionprocessing instance;


  private Visionprocessing() {

  }

  @Override
  public void periodic() {

  }

  private void getData(){
    double[] data= NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
  }

  

  public static Visionprocessing getInstance(){
    if (instance != null){
        return instance;
    }
    else{
        instance = new Visionprocessing();
        return instance;
    }
  }
}
