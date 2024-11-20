package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Config;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeCommand2 extends Command{
    private ShooterSubsystem climberSubsystem = (ShooterSubsystem) Config.active.getShooter().get();
    
    public IntakeCommand2(){
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
