package frc.robot.subsystems.visionAutonomous;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Config;

public class TankDrivePoseEstimator {
	public DifferentialDrivePoseEstimator m_poseEstimator;

	public static TankDrivePoseEstimator instance;

	private TankDrivePoseEstimator() {
		assert Config.drive().isSwerve();

		m_poseEstimator = new DifferentialDrivePoseEstimator(
				Config.drive().getDifferentialKinematics().get(),
				Gyro.getInstance().getRotation2d(),
				Config.drive().getLeftEncoderPos(),
				Config.drive().getRightEncoderPos(),
				new Pose2d(),
				VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
				VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
	}

	public void updatePoseEstimator() {
		m_poseEstimator.update(
				Gyro.getInstance().getRotation2d(),
				Config.drive().getLeftEncoderPos(),
				Config.drive().getRightEncoderPos());
	}

	// Atention not clear witch values are releveant for the pose reseting
	private void setPos(Rotation2d rot, double weelLeftPosition, double weelRigthPosition, Pose2d pos) {
		m_poseEstimator.resetPosition(rot, weelLeftPosition, weelRigthPosition, pos);
	}

	private void visionProcessing() {
		// Pose3d visionMeasurement3d =
		// objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

		// Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
		// Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
	}

	public static TankDrivePoseEstimator getInstance() {
		if (instance == null) {
			instance = new TankDrivePoseEstimator();
		}
		return instance;
	}

}
