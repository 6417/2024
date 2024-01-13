package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Visionprocessing extends SubsystemBase {

    static Visionprocessing instance;
  
  //final DoubleSubscriber ySub;
  //double prev;

  private Visionprocessing() {

  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
