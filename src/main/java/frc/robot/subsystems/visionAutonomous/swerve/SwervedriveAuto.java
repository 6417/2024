package frc.robot.subsystems.visionAutonomous.swerve;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.abstraction.baseClasses.BAutoHandler;
import frc.robot.commands.SwervedriveAutoCommand;
import frc.robot.subsystems.drive.getSwerveAutonomousTrj;

public class SwervedriveAuto extends BAutoHandler {

	ChassisSpeeds speeds = new ChassisSpeeds();

	private HolonomicDriveController controller = new HolonomicDriveController(
			new PIDController(5, 0.1, 0.1), new PIDController(1, 0.1, 0.1),
			new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));

	public SwervedriveAuto() {
	}

	@Override
	public ChassisSpeeds getVelocitiesAtTimepoint(Trajectory tra, double t) {
		Trajectory.State goal = tra.sample(t);
		Rotation2d rot = goal.poseMeters.getRotation();
		speeds = controller.calculate(Config.drive().getPos(), goal, rot);
		return speeds;
	}

	@Override
	public Command getAutoCommand() {
		Pose2d firstApriltag = new Pose2d(15, 5.5, new Rotation2d(0));
		Pose2d test = new Pose2d(1, 0, new Rotation2d(0));

		Trajectory tra = getSwerveAutonomousTrj.getInstance().createTrajectory(firstApriltag, 1);

		Trajectory testtra = getSwerveAutonomousTrj.getInstance().createTrajectory(test, 2);
		return new SwervedriveAutoCommand(testtra);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("SpeedsX", () -> speeds.vxMetersPerSecond, null);
		builder.addDoubleProperty("SpeedsY", () -> speeds.vyMetersPerSecond, null);
		builder.addDoubleProperty("SpeedsRot", () -> speeds.omegaRadiansPerSecond, null);

		builder.addDoubleProperty("ActualX", () -> Config.drive().getPos().getX(), null);
		builder.addDoubleProperty("ActualY", () -> Config.drive().getPos().getY(), null);
		builder.addDoubleProperty("actualRot", () -> Config.drive().getPos().getRotation().getDegrees(), null);
	}
}
