package frc.robot.commands.drive.commands_2024;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.fridowpi.command.FridoCommand;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BSwerveModule;
import frc.robot.subsystems.drive.swerve_2024.SwerveDrive2024;

public class ZeroAngleMotors extends ParallelCommandGroup {
	private final SwerveDrive2024 drive;

	public ZeroAngleMotors() {
		assert Config.drive() instanceof SwerveDrive2024: "Not implemented for " + Config.drive().getClass();
		drive = (SwerveDrive2024) Config.drive();
		addRequirements(Config.drive());

		// Build sequential command group
		List<SequentialCommandGroup> commands = new ArrayList<>();
		drive.forEachModule(module -> {
			SequentialCommandGroup commandGroup = new SequentialCommandGroup(
					new ZeroEncodersApprox(module),
					new ZeroEncodersFineTune(module));
			commands.add(commandGroup);
		});
		addCommands(commands.toArray(SequentialCommandGroup[]::new));
	}

	private class ZeroEncodersApprox extends FridoCommand {
		private double currentSetPoint;
		private BSwerveModule module;

		public ZeroEncodersApprox(BSwerveModule module) {
			this.module = module;
		}

		@Override
		public void initialize() {
			module.setEncoderZeroedFalse();
			module.stopAllMotors();
			currentSetPoint = module.getRotationEncoderTicks() + Constants.SwerveDrive.Swerve2024.zeroingSpeed;
		}

		@Override
		public void execute() {
			module.setDesiredRotationMotorTicks(currentSetPoint);
			currentSetPoint += Constants.SwerveDrive.Swerve2024.zeroingSpeed;
		}

		@Override
		public void end(boolean interrupted) {
			module.stopAllMotors();
		}

		@Override
		public boolean isFinished() {
			return module.isAtZero();
		}
	}

	private class ZeroEncodersFineTune extends Command {
		private double currentSetPoint;
		private BSwerveModule module;

		public ZeroEncodersFineTune(BSwerveModule module) {
			this.module = module;
		}

		@Override
		public void initialize() {
			module.setEncoderZeroedFalse();
			module.stopAllMotors();
			currentSetPoint = module.getRotationEncoderTicks()
					- Constants.SwerveDrive.Swerve2024.zeroingSpeed * Constants.SwerveDrive.Swerve2024.finetuningZeroFactor;
			while (module.isAtZero()) {
				module.setDesiredRotationMotorTicks(currentSetPoint);
				currentSetPoint -= Constants.SwerveDrive.Swerve2024.zeroingSpeed * Constants.SwerveDrive.Swerve2024.finetuningZeroFactor;
			}
			currentSetPoint = module.getRotationEncoderTicks();
		}

		@Override
		public void execute() {
			module.setDesiredRotationMotorTicks(currentSetPoint);
			currentSetPoint += Constants.SwerveDrive.Swerve2024.zeroingSpeed * Constants.SwerveDrive.Swerve2024.finetuningZeroFactor;
			if (module.isAtZero()) {
				module.setRotationEncoderTicks(module.getConfig().absoluteEncoderZeroPosition);
			}
		}

		@Override
		public boolean isFinished() {
			return module.hasEncoderBeenZeroed();
		}

		@Override
		public void end(boolean interrupted) {
			module.stopAllMotors();
		}
	}
}
