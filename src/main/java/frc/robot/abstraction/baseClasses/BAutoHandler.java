package frc.robot.abstraction.baseClasses;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.fridowpi.module.Module;

/**
 * BAutoHandler
 */
public abstract class BAutoHandler extends Module {

	abstract public Command getAutoCommand(Trajectory tra);

	abstract public ChassisSpeeds getVelocitiesAtTimepoint(Trajectory tra, double t);
}
