package frc.robot.subsystems.visionAutonomous;

import static frc.robot.Utils.logerr;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Config;
import frc.robot.Constants;

public class SwerveDrivePoseEstimator extends edu.wpi.first.math.estimator.SwerveDrivePoseEstimator {

	private Timer timer;

	public SwerveDrivePoseEstimator(SwerveDriveKinematics kinematics,
			SwerveModulePosition[] modulePositions,
			Rotation2d navxRotation, Pose2d initialPose) {
		super(
				kinematics,
				navxRotation,
				modulePositions,
				initialPose);

		if (!Config.drive().isSwerve()) {
			logerr("Should be a swerve for SwervDrivePoseEstimator");
			return;
		}

		timer = new Timer();
		timer.start();
	}

	public static SwerveDrivePoseEstimator fromFieldPos(double[] pos) {
		return new SwerveDrivePoseEstimator(
				Config.drive().getSwerveKinematics().get(),
				Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS,
				FridoNavx.getInstance().getRotation2d(),
				new Pose2d(pos[0], pos[1], new Rotation2d(pos[5])));
	}

	private double get_dist_to_apriltag() {
		double[] pos = Visionprocessing.getInstance().getData();
		return Math.sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
	}

	private double stand_div_vision(double x) {
		final double maxDist = 12;
		final double div_at_max_dis = 8.5;
		final double divmin = 0.05;
		return Math.tanh(x / maxDist) * (div_at_max_dis / Math.tanh(1) + divmin);
	}

	public void update() {
		Rotation2d gyroAngle = FridoNavx.getInstance().getRotation2d();
		updateWithTime(timer.get(), gyroAngle,
				Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS);

		int t = Visionprocessing.getInstance().validTarget();

		if (t == 1) {
			double[] visionPosition = Visionprocessing.getInstance().getFieldPos();
			Pose2d pos = new Pose2d(visionPosition[0], visionPosition[1], gyroAngle);
			double dist = get_dist_to_apriltag();
			double standDiv = stand_div_vision(dist);
			addVisionMeasurement(pos, timer.get(), VecBuilder.fill(standDiv, standDiv, 0.5));
			// Gyro.getInstance().reset();
			// if (visionPosition[5] < 0){
			// visionPosition[5] = visionPosition[5]; //360-
			// }
		}
	}

	public void reset_odometry() {
		resetPosition(FridoNavx.getInstance().getRotation2d(),
				Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS, new Pose2d(0, 0, new Rotation2d(0)));
	}
}
