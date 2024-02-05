package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;

public class SwervdriveAutoTrajectory {

    private SwervdriveAutoTrajectory(){

    }
    
    private void getConstraint(){
    }

    TrajectoryConfig trajectoryconfig = new TrajectoryConfig(
        Constants.Swervedrive.Drive.kMaxSpeedMetersPerSecond, 
        Constants.Swervedrive.Drive.kMaxAccelerationMetersPerSecondSquared);
}
