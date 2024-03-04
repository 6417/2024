package frc.robot.abstraction.interfaces;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.fridowpi.module.IModule;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;

/**
 * IMotorSet: Common interface for differential motor sets
 */
public interface IMotorSet extends IModule {

	public FridolinsMotor getMotor(MotorRole role);

	public void invert(MotorRole role);

	public void setIdleMode(IdleMode mode);

	public IdleMode getIdleMode();

	public void setVolts(Measure<Voltage> leftVolts, Measure<Voltage> rightVolts);

	public void stopAll();
}
