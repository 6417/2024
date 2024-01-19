package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.drive.Tankdrive;

public class Tankdrive_poseestimator {
    public DifferentialDrivePoseEstimator m_poseEstimator;

    public static Tankdrive_poseestimator instance;

    private Tankdrive_poseestimator(){
        m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            Tankdrive.getInstance().m_kinematics,
            Gyro.getInstance().getRotation2d(),
            Tankdrive.getInstance().getLeftEncoderPos(),
            Tankdrive.getInstance().getRigthEncoderPos(),
            new Pose2d(),
            VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
            VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    private void updatePoseEstimator(){
        m_poseEstimator.update(
        Gyro.getInstance().getRotation2d(), Tankdrive.getInstance().getLeftEncoderPos(), Tankdrive.getInstance().getRigthEncoderPos());
    }

    

    private void visionProcessing(){
        //Pose3d visionMeasurement3d =
        //objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

        // Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
        //Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
    }

    public static Tankdrive_poseestimator getInstance(){
        if (instance == null){
            instance = new Tankdrive_poseestimator();
        }
        return instance;
    }

}