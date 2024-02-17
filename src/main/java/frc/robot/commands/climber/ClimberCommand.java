// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.fridowpi.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends SequentialCommandGroup {

  public ClimberCommand(ClimberSubsystem subsystem) {
    addCommands(
        new ClimberPid(Constants.Climber.ausfahrBereich),
        new ClimberPid(Constants.Climber.zielPosition)
    );
  }
}
