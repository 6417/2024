package frc.robot.subsystems.drive.tankdrive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.DirectionType;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.abstraction.baseClasses.BMotorSet;

/** A small helper class for a set of four motors **/
public class MotorSet extends BMotorSet {
	FridolinsMotor leftMaster;
	FridolinsMotor rightMaster;
	FridolinsMotor leftFollower;
	FridolinsMotor rightFollower;

	// Default to Coast
	IdleMode idleMode = IdleMode.kCoast;

	public enum MotorRole {
		LeftMaster, RightMaster,
		LeftFollower, RightFollower
	}

	public MotorSet(
			FridolinsMotor leftMaster, FridolinsMotor rightMaster,
			FridolinsMotor leftFollower, FridolinsMotor rightFollower) {
		this.leftMaster = leftMaster;
		this.rightMaster = rightMaster;
		this.leftFollower = leftFollower;
		this.rightFollower = rightFollower;

		// Default to following without inversion
		this.rightFollower.follow(this.rightMaster, DirectionType.followMaster);
		this.leftFollower.follow(this.leftMaster, DirectionType.followMaster);
		setIdleMode(idleMode);
	}

	/* Invert motor with a given role, while keeping the others the same */
	@Override
	public void invert(MotorRole role) {
		var m = getMotor(role);
		switch (role) {
			// If master, invert it but revert direction of the follower
			case LeftMaster:
				m.setInverted(true);
				/* Fallthrough */
			case LeftFollower:
				leftFollower.follow(m, DirectionType.invertMaster);
				break;
			case RightMaster:
				m.setInverted(true);
				/* Fallthrough */
			case RightFollower:
				rightFollower.follow(m, DirectionType.invertMaster);
				break;
		}
		getMotor(role).setInverted(true);
	}

	@Override
	public FridolinsMotor getMotor(MotorRole motor) {
		switch (motor) {
			case LeftMaster:
				return leftMaster;
			case RightMaster:
				return rightMaster;
			case LeftFollower:
				return leftFollower;
			case RightFollower:
				return leftFollower;
		}
		// Not possible, as long as all enum variants are handled above
		return null;
	}

	@Override
	public void setIdleMode(IdleMode mode) {
		leftMaster.setIdleMode(mode);
		rightMaster.setIdleMode(mode);
		idleMode = mode;
	}

	@Override
	public IdleMode getIdleMode() {
		return idleMode;
	}

	@Override
	public void setVolts(Measure<Voltage> leftVolts, Measure<Voltage> rightVolts) {
		leftMaster.set(leftVolts.in(Volts));
		rightMaster.set(rightVolts.in(Volts));
	}
}
