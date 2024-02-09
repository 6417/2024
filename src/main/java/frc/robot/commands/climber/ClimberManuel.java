// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.fridowpi.motors.FridolinsMotor.PidType;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimberManuel extends Command {
    boolean directionIsUp;

    public ClimberManuel(boolean directionIsUp) {
        this.directionIsUp = directionIsUp;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ClimberSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (ClimberSubsystem.seilZiehMotorLinks.pidAtTarget()) {
            if (directionIsUp) {
                ClimberSubsystem.seilZiehMotorLinks.setPidTarget(ClimberSubsystem.seilZiehMotorLinks.getPidTarget()
                        + Constants.Climber.manualClimberMovementSpeed, PidType.position);
            } else {
                ClimberSubsystem.seilZiehMotorLinks.setPidTarget(ClimberSubsystem.seilZiehMotorLinks.getPidTarget()
                        - Constants.Climber.manualClimberMovementSpeed, PidType.position);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
