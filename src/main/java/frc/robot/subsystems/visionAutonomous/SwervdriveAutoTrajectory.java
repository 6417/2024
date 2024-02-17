package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Config;

public class SwervdriveAutoTrajectory {

    private SwervdriveAutoTrajectory(){

    }
    
    private void getConstraint(){
    }

    TrajectoryConfig trajectoryconfig = new TrajectoryConfig(
        Config.data().auto().maxVelocity(),
        Config.data().auto().maxAcceleration());
}
