package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.fridolib.QuickCmd;
import frc.robot.Config;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberAutoClimb extends SequentialCommandGroup{

    public ClimberAutoClimb(){
        var climberSubsystem = Config.active.getClimber().get();
        addRequirements(climberSubsystem);
        addCommands(
            QuickCmd.withInit(climberSubsystem::release),
            new WaitCommand(3),
            new RetractClimber()
        );
    }
    
}
