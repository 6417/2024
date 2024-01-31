package frc.robot.subsystems.vision_autonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import frc.robot.Constants;

public class Swervdrive_auto_trajectory {

    private Swervdrive_auto_trajectory(){

    }
    
    private void getConstraint(){
    }

    TrajectoryConfig trajectoryconfig = new TrajectoryConfig(
        Constants.Swervedrive.Drive.kMaxSpeedMetersPerSecond, 
        Constants.Swervedrive.Drive.kMaxAccelerationMetersPerSecondSquared);
}
