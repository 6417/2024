package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwervDrivePoseEstimator extends SubsystemBase {
  SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
  
  private static SwervDrivePoseEstimator instance;

  public SwervDrivePoseEstimator() {}

  public void update(Pose2d pose){
    //swerveDrivePoseEstimator.update(pose, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static SwervDrivePoseEstimator getInstance(){
    if (instance == null){
      instance = new SwervDrivePoseEstimator();
    }
    return instance;
  }
}
