package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.abstraction.baseClasses.BClimber;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractClimber extends Command {
    private ClimberSubsystem climberSubsystem = (ClimberSubsystem) Config.active.getClimber().get();

    public RetractClimber() {
        addRequirements(climberSubsystem);
        System.out.println("Retractclimber created ##########################################");
    }

    @Override
    public void initialize() {
        System.out.println("Retract climber initialized %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        climberSubsystem.setSpeed(0.1);
    }

    @Override
    public boolean isFinished() {
        return (climberSubsystem.getSeilMotorLinks().getEncoderTicks() >= Constants.Climber.maxExtentionEncoderTicks ||
                climberSubsystem.getSeilMotorRechts().getEncoderTicks() >= Constants.Climber.maxExtentionEncoderTicks);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stopMotors();
    }
}
