// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.fridowpi.command.ParallelCommandGroup;
import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/** Releases the climber through the servo motor. */
public class ClimberRelease extends SequentialCommandGroup {

	public ClimberRelease(ClimberSubsystem subsystem) {
		addRequirements(subsystem);
		addCommands(
			new ParallelCommandGroup(
				new InstantCommand(() -> subsystem.federLoslassMotorLinks.setAngle(Constants.Climber.maxServoPos)),
				new InstantCommand(() -> subsystem.federLoslassMotorRechts.setAngle(Constants.Climber.maxServoPos))
			),
			new ClimberPid(subsystem, Constants.Climber.ausfahrBereich)
		);
	}
}
