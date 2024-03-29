package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;

public class getAutonomousTrajectory extends SubsystemBase {

	public static getAutonomousTrajectory instance;

	private TrajectoryConfig getTrajectoryConfig() {
		assert !Config.drive().isSwerve();

		var voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.Testchassis.ksVolts,
						Constants.Testchassis.kvVoltSevondsPerMeter,
						Constants.Testchassis.kaVoltSecondsSquaredPerMeter),
				Config.drive().getDifferentialKinematics().get(), 10);

		TrajectoryConfig config = new TrajectoryConfig(Constants.Testchassis.PathWeaver.kMaxVMetersPerSecond,
				Constants.Testchassis.PathWeaver.kMaxAccMetersPerSecond)
				.setKinematics(Config.drive().getDifferentialKinematics().get())
				.addConstraint(voltageConstraint);

		return config;
	}

	public Trajectory get_abs_trajectory(Pose2d endPose) {
		TrajectoryConfig conf = getTrajectoryConfig();
		List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

		Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
				Config.drive().getPos(),
				list_Translation2d,
				endPose,
				conf);

		return new_trajectory;
	}

	public Trajectory get_rel_Trajectory(Pose2d endPose) {
		TrajectoryConfig conf = getTrajectoryConfig();
		List<Translation2d> list_Translation2d = new ArrayList<Translation2d>();

		Translation2d endtranslation = Config.drive().getPos()
				.getTranslation().plus(endPose.getTranslation());
		endPose = new Pose2d(endtranslation, endPose.getRotation());
		Trajectory new_trajectory = null;

		new_trajectory = TrajectoryGenerator.generateTrajectory(
				Config.drive().getPos(),
				list_Translation2d,
				endPose,
				conf);

		return new_trajectory;
	}

	public Trajectory createTrajectory(Pose2d endPose, int type) {
		Trajectory trajectory;

		if (type == 1) {
			trajectory = get_abs_trajectory(endPose);
		} else if (type == 2) {
			trajectory = get_rel_Trajectory(endPose);
		} else {
			trajectory = get_abs_trajectory(endPose); // shit default
		}
		return trajectory;
	}

	public static getAutonomousTrajectory getInstance() {
		if (instance == null) {
			instance = new getAutonomousTrajectory();
		}

		return instance;
	}
}
