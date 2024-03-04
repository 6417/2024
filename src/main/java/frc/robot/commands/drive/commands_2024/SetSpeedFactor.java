package frc.robot.commands.drive.commands_2024;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class SetSpeedFactor extends Command {
    private double speedFactor;
    private static SubsystemBase speedFactorCommandRequirement = new SubsystemBase() {};
    public SetSpeedFactor(double speedFactor) {
        this.speedFactor = speedFactor;
        addRequirements(speedFactorCommandRequirement);
    }

    @Override
    public void initialize() {
        Config.drive().setSpeedFactor(speedFactor);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
