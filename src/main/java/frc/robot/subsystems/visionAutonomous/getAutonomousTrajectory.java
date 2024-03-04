package frc.robot.subsystems.visionAutonomous;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;

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
import frc.robot.Config;
import frc.robot.Constants;

public class getAutonomousTrajectory {

	public static getAutonomousTrajectory instance;

	private TrajectoryConfig getTrajectoryConfig() {
		assert !Config.drive().isSwerve();

		final var voltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(Constants.Testchassis.ksVolts,
						Constants.Testchassis.kvVoltSevondsPerMeter,
						Constants.Testchassis.kaVoltSecondsSquaredPerMeter),
				Config.drive().getDifferentialKinematics().get(), 10);

		final TrajectoryConfig config = new TrajectoryConfig(Constants.Testchassis.PathWeaver.kMaxVMetersPerSecond,
				Constants.Testchassis.PathWeaver.kMaxAccMetersPerSecond)
				.setKinematics(Config.drive().getDifferentialKinematics().get())
				.addConstraint(voltageConstraint);


		return config;
	}

	private RamseteCommand getTrajectory() {
		assert !Config.drive().isSwerve() : "Swerve is not yet implemented";

		TrajectoryConfig conf = getTrajectoryConfig();
		List<Translation2d> list_translationd2 = new ArrayList<Translation2d>();

		Trajectory new_trajectory = TrajectoryGenerator.generateTrajectory(
				TankDrivePoseEstimator.getInstance().m_poseEstimator.getEstimatedPosition(),
				list_translationd2,
				TankDrivePoseEstimator.getInstance().m_poseEstimator.getEstimatedPosition()
						.plus(new Transform2d(new Translation2d(1, 0), new Rotation2d(0))),
				conf);

		var controller = new RamseteController(
				Config.data().auto().kRamseteB().in(Meters),
				Config.data().auto().kRamseteZeta().in(Seconds));

		var pidLeft = new PIDController(
				Config.data().pid().driveLeft().kP,
				Config.data().pid().driveLeft().kI,
				Config.data().pid().driveLeft().kD);

		var pidRight = new PIDController(
				Config.data().pid().driveRight().kP,
				Config.data().pid().driveRight().kI,
				Config.data().pid().driveRight().kD);

		var feedforward = new SimpleMotorFeedforward(
				Config.data().auto().ksVolts().in(Meters),
				Config.data().auto().kvVolts().in(MetersPerSecond),
				Config.data().auto().kaVolts().in(MetersPerSecondPerSecond));

		var command = new RamseteCommand(
				new_trajectory,
				Config.drive()::getPos,
				controller,
				feedforward,
				Config.drive().getDifferentialKinematics().get(),
				() -> Config.drive().getDifferentialWheelSpeeds().get(),
				pidLeft,
				pidRight,
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
