package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.fridolib.RobotPresets.RobotPreset;
import frc.robot.Config;
import frc.robot.Constants;

public class getAutonomousTrajectory {

	public static getAutonomousTrajectory instance;

	private TrajectoryConfig getTrajectoryConfig() {
		assert !Config.drive().isSwerve();

		final var voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
						Constants.Testchassi.kvVoltSevondsPerMeter,
						Constants.Testchassi.kaVoltSecondsSquaredPerMeter),
				Config.drive().getDifferentialKinematics().get(), 10);

		final TrajectoryConfig config = new TrajectoryConfig(Constants.Testchassi.PathWeaver.kMaxVMetersPerSecond,
				Constants.Testchassi.PathWeaver.kMaxAccMetersPerSecond)
				.setKinematics(Config.drive().getDifferentialKinematics().get())
				.addConstraint(voltageConstraint);

		return config;
	}

	private RamseteCommand getTrajectory() {
		assert !Config.drive().isSwerve();

		TrajectoryConfig conf = getTrajectoryConfig();
		List<Translation2d> list_translationd2 = new ArrayList<Translation2d>();

		Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
				TankDrivePoseEstimator.getInstance().m_poseEstimator.getEstimatedPosition(),
				list_translationd2,
				TankDrivePoseEstimator.getInstance().m_poseEstimator.getEstimatedPosition()
						.plus(new Transform2d(new Translation2d(1, 0), new Rotation2d(0))),
				conf);

		RamseteCommand command = new RamseteCommand(
				new_trajectory,
				Config.drive()::getPos,
				new RamseteController(Constants.Testchassi.PathWeaver.kRamsetB,
						Constants.Testchassi.PathWeaver.kRamseteZeta),
				new SimpleMotorFeedforward(Constants.Testchassi.ksVolts,
						Constants.Testchassi.kvVoltSevondsPerMeter,
						Constants.Testchassi.kaVoltSecondsSquaredPerMeter),
				Config.drive().getDifferentialKinematics().get(),
				() -> Config.drive().getDifferentialWheelSpeeds().get(),
				new PIDController(Constants.Testchassi.kPDriveVel, 0, 0),
				new PIDController(Constants.Testchassi.kPDriveVel, 0, 0),
				Config.drive()::setVolts,
				Config.drive());

		return command;
	}

	public RamseteCommand start_command() {
		RamseteCommand command = getTrajectory();
		// command.schedule();
		return command;
	}

	public static getAutonomousTrajectory getInstance() {
		if (instance == null) {
			instance = new getAutonomousTrajectory();
		}
		return instance;
	}
}
