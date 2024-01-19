package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swervdrive_poseestimator extends SubsystemBase {
  SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
  
  private static Swervdrive_poseestimator instance;

  public Swervdrive_poseestimator() {}

  public void update(Pose2d pose){
    //swerveDrivePoseEstimator.update(pose, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static Swervdrive_poseestimator getInstance(){
    if (instance == null){
      instance = new Swervdrive_poseestimator();
    }
    return instance;
  }
}
