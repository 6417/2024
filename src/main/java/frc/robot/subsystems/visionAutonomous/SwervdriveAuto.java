package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.commands.SwerdriveAutoCommand;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;

public class SwervdriveAuto {

    static SwervdriveAuto instance;
    
    private HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, 
        new TrapezoidProfile.Constraints(6.28, 3.14)));

    private SwervdriveAuto(){

    }

    public ChassisSpeeds getVelocitis(Trajectory tra, double t){
        Pose2d robotPose = SwervDrivePoseEstimator.getInstance().getPoseEstimator().getEstimatedPosition();
        Trajectory.State goal = tra.sample(t);
        Rotation2d rot = goal.poseMeters.getRotation();
        ChassisSpeeds speeds = controller.calculate(robotPose, goal, rot);
        return speeds;
    }

    public void startCommand(){
        Pose2d firstApriltag = new Pose2d(15, 5.5, new Rotation2d(0));
        Pose2d test = new Pose2d(1,0,new Rotation2d(0));

        Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(firstApriltag, 1);
        
        Trajectory testtra = getSwerveAutonomousTrj.getInstance().createTrajectory(test, 2);
        SwerdriveAutoCommand command = new SwerdriveAutoCommand(testtra);
        command.schedule();
    }

    public static SwervdriveAuto getInstance(){
        if (instance == null){
            instance = new SwervdriveAuto();
        }
        return instance;
    }
}
