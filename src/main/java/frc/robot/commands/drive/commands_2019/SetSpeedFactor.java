package frc.robot.commands.drive.commands_2019;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.swerve_2019.SwerveDrive2019;

public class SetSpeedFactor extends Command {
    private double speedFactor;
    private static SubsystemBase speedFactorCommandRequirement = new SubsystemBase() {};
    public SetSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
        addRequirements(speedFactorCommandRequirement);
    }

    @Override
    public void initialize() {
        SwerveDrive2019.getInstance().setSpeedFactor(speedFactor);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
