package frc.robot.commands;

import java.text.spi.DecimalFormatSymbolsProvider;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;

public class BreakCommand extends Command {
    DriveBase m_drive_subsystem;

    public BreakCommand(DriveBase drive_subsystem) {
        m_drive_subsystem = drive_subsystem;
    }

    @Override
    public void initialize() {
        m_drive_subsystem.brake();
    }
}
