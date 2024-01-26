package frc.robot.subsystems.vision_autonomous;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionprocessing extends SubsystemBase {

    static Visionprocessing instance;

  private Visionprocessing() {
  }

  @Override
  public void periodic() {
  }

  private double[] getData(){
    double[] data= NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    return data;
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
}//setNeutralMode(enutralmodevalue.brake/coast)
