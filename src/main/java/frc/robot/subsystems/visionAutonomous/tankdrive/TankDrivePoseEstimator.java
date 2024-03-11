package frc.robot.subsystems.visionAutonomous.tankdrive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Config;
import frc.fridowpi.sensors.FridoNavx;

public class TankDrivePoseEstimator extends DifferentialDrivePoseEstimator {
	public static TankDrivePoseEstimator instance;

	public TankDrivePoseEstimator(Pose2d pos) {
		super(Config.drive().getDifferentialKinematics().get(),
				FridoNavx.getInstance().getRotation2d(),
				Config.drive().getLeftEncoderPos(),
				Config.drive().getRightEncoderPos(),
				pos,
				VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
				VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
		assert Config.drive().isSwerve();
	}

	public void update() {
		update(
				FridoNavx.getInstance().getRotation2d(),
				Config.drive().getLeftEncoderPos(),
				Config.drive().getRightEncoderPos());
	}

	public void reset() {
	}

	// Atention not clear witch values are releveant for the pose reseting
	private void setPos(Rotation2d rot, double weelLeftPosition, double weelRigthPosition, Pose2d pos) {
		resetPosition(rot, weelLeftPosition, weelRigthPosition, pos);
	}

	private void visionProcessing() {
		// Pose3d visionMeasurement3d =
		// objectToRobotPose(m_objectInField, m_robotToCamera, m_cameraToObjectEntry);

		// Convert robot pose from Pose3d to Pose2d needed to apply vision measurements.
		// Pose2d visionMeasurement2d = visionMeasurement3d.toPose2d();
	}
}
