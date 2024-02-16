package frc.robot.subsystems.vision_autonomous;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveDriveBrake;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwervdriveAuto {

    static SwervdriveAuto instance;
    
    private HolonomicDriveController controller = new HolonomicDriveController(
        new PIDController(1, 0, 0), new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, 
        new TrapezoidProfile.Constraints(6.28, 3.14)));

    private SwervdriveAuto(){

    }

    public ChassisSpeeds getVelocitis(Trajectory tra, double t){
        Pose2d robotPose = Swervdrive_poseestimator.getInstance().swerveDrivePoseEstimator.getEstimatedPosition();
        Trajectory.State goal = tra.sample(t);
        Rotation2d rot = goal.poseMeters.getRotation();
        ChassisSpeeds speeds = controller.calculate(robotPose, goal, rot);
        return speeds;
    }

    public static SwervdriveAuto getInstance(){
        if (instance == null){
            instance = new SwervdriveAuto();
        }
        return instance;
    }
}
