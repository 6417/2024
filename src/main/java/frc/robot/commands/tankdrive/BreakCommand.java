package frc.robot.commands.tankdrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.abstraction.baseClasses.BDrive;

public class BreakCommand extends Command {
    BDrive m_drive_subsystem;

    public BreakCommand(BDrive drive_subsystem) {
        m_drive_subsystem = drive_subsystem;
    }

    @Override
    public void initialize() {
        m_drive_subsystem.setIdleMode(IdleMode.kBrake);
    }
}
