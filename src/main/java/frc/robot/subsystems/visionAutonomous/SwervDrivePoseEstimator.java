package frc.robot.subsystems.visionAutonomous;

import static frc.robot.Utils.logerr;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveDrive;

public class SwervDrivePoseEstimator extends SubsystemBase {
	private SwerveDrivePoseEstimator swerveDrivePoseEstimator;

	private static SwervDrivePoseEstimator instance;
	private Timer timer;
	private BSwerveDrive drive;

	public SwervDrivePoseEstimator() {
		if (!Config.drive().isSwerve()) {
			logerr("Should be a swerve for SwervDrivePoseEstimator");
			return;
		}
		drive = (BSwerveDrive) Config.drive();
		double[] pos = Visionprocessing.getInstance().getFieldPos();

		swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(drive.getSwerveKinematics().get(),
				FridoNavx.getInstance().getRotation2d(), Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS,
				new Pose2d(pos[0], pos[1], new Rotation2d(Units.degreesToRadians(pos[5]))));

		timer = new Timer();
		timer.start();
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
		swerveDrivePoseEstimator.updateWithTime(timer.get(), gyroAngle,
				Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS);

		int t = Visionprocessing.getInstance().validTarget();

		if (t == 1) {
			double[] visionPosition = Visionprocessing.getInstance().getFieldPos();
			Pose2d pos = new Pose2d(visionPosition[0], visionPosition[1], gyroAngle);
			double dist = get_dist_to_apriltag();
			double standDiv = stand_div_vision(dist);
			swerveDrivePoseEstimator.addVisionMeasurement(pos, timer.get(), VecBuilder.fill(standDiv, standDiv, 0.5));
			// Gyro.getInstance().reset();
			// if (visionPosition[5] < 0){
			// visionPosition[5] = visionPosition[5]; //360-
			// }
		}
	}

	public void reset_odometry() {
		swerveDrivePoseEstimator.resetPosition(FridoNavx.getInstance().getRotation2d(),
				Constants.SwerveDrive.Swerve2024.SWERVE_MODULE_POSITIONS, new Pose2d(0, 0, new Rotation2d(0)));
	}

	@Override
	public void periodic() {
		update();
	}

	public SwerveDrivePoseEstimator getPoseEstimator() {
		return swerveDrivePoseEstimator;
	}

	public static SwervDrivePoseEstimator getInstance() {
		if (instance == null) {
			instance = new SwervDrivePoseEstimator();
		}
		return instance;
	}
}
