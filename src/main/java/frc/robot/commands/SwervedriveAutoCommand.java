package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;

public class SwervedriveAutoCommand extends Command {

	Timer timer = new Timer();
	Trajectory trajectory;

	public SwervedriveAutoCommand(Trajectory tra) {
		trajectory = tra;
	}

	@Override
	public void initialize() {
		timer.start();
	}

	@Override
	public void execute() {
		double t = timer.get();
		ChassisSpeeds speeds = Config.active.getAuto().get().getVelocitiesAtTimepoint(trajectory, t);
		Config.drive().drive(speeds);
	}

	@Override
	public void end(boolean interrupted) {
	}

	@Override
	public boolean isFinished() {
		// return timer.hasElapsed(trajectory.getTotalTimeSeconds());
		return false;
	}
}
