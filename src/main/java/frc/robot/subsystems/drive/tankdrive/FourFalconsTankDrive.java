package frc.robot.subsystems.drive.tankdrive;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.fridowpi.motors.FridoFalcon500;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.robot.Config;
import frc.robot.Controls;
import frc.robot.abstraction.baseClasses.BMotorSet;
import frc.robot.abstraction.baseClasses.BTankDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.drive.tankdrive.MotorSet.MotorRole;
import frc.robot.subsystems.visionAutonomous.TankDrivePoseEstimator;

public class FourFalconsTankDrive extends BTankDrive {

	SysIdRoutineLog log;

	public BMotorSet motors;
	public DifferentialDriveKinematics m_kinematics;
	private DifferentialDrive differentialDrive;

	public FourFalconsTankDrive(
			int leftMaster,
			int rightMaster,
			int leftFollower,
			int rightFollower) {

		motors = new MotorSet(
			new FridoFalcon500(leftMaster),
			new FridoFalcon500(rightMaster),
			new FridoFalcon500(leftFollower),
			new FridoFalcon500(rightFollower));

		motors.invert(MotorRole.LeftMaster);

		differentialDrive = new DifferentialDrive(
			motors.getMotor(MotorRole.LeftMaster)::set,
			motors.getMotor(MotorRole.RightMaster)::set);

		m_kinematics = new DifferentialDriveKinematics(Config.data().wheelPerimeter());

		setDefaultCommand(new DriveCommand(this));
	}

	// Mutable holders for unit-safe values, persisted to avoid reallocation.
	final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
	final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
	final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

	private final SysIdRoutine routine = new SysIdRoutine(
			new SysIdRoutine.Config(),
			new SysIdRoutine.Mechanism(volts -> setVolts(volts.in(Volts), volts.in(Volts)),
					log -> {
						// Record a frame for the left motors. Since these share an encoder, we consider
						// the entire group to be one motor.
						log.motor("drive-left")
								.voltage(
										m_appliedVoltage.mut_replace(
												motors.getMotor(MotorRole.LeftMaster).get() * RobotController.getBatteryVoltage(), Volts))
								.linearPosition(m_distance.mut_replace(getLeftEncoderPos(), Meters))
								.linearVelocity(
										m_velocity.mut_replace(getDifferentialWheelSpeeds().get().leftMetersPerSecond, MetersPerSecond));
						// Record a frame for the right motors. Since these share an encoder, we
						// consider the entire group to be one motor.
						log.motor("drive-right")
								.voltage(
										m_appliedVoltage.mut_replace(
												motors.getMotor(MotorRole.RightMaster).get() * RobotController.getBatteryVoltage(), Volts))
								.linearPosition(m_distance.mut_replace(getRightEncoderPos(), Meters))
								.linearVelocity(
										m_velocity.mut_replace(getDifferentialWheelSpeeds().get().rightMetersPerSecond, MetersPerSecond));
					},
					this));

	@Override
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return routine.quasistatic(direction);
	}

	@Override
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return routine.dynamic(direction);
	}

	@Override
	public void periodic() {
	}

	@Override
	public double getLeftEncoderPos() {
		return motors.getMotor(MotorRole.LeftMaster).getEncoderTicks();
	}

	@Override
	public double getRightEncoderPos() {
		return motors.getMotor(MotorRole.RightMaster).getEncoderTicks();
	}

	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds() {
		return Optional.of(new DifferentialDriveWheelSpeeds(
				motors.getMotor(MotorRole.LeftMaster).get() * 10 * Config.data().encoderToMeters(),
				motors.getMotor(MotorRole.RightMaster).get() * -10 * Config.data().encoderToMeters()));
	}

	@Override
	public Optional<DifferentialDriveKinematics> getDifferentialKinematics() {
		return Optional.of(m_kinematics);
	}

	@Override
	public void setVolts(double leftvolts, double rigthvolts) {
		motors.getMotor(MotorRole.LeftMaster).setVoltage(leftvolts);
		motors.getMotor(MotorRole.RightMaster).setVoltage(-rigthvolts);
		// m_drive.feed();
	}

	@Override
	public Pose2d getPos() {
		return TankDrivePoseEstimator.getInstance().m_poseEstimator.getEstimatedPosition();
	}

	@Override
	public final boolean isSwerve() {
		return false;
	}

	@Override
	public void brake() {
		motors.setIdleMode(IdleMode.kBrake);
	}

	@Override
	public void release_brake() {
		motors.setIdleMode(IdleMode.kCoast);
	}

	@Override
	public void drive(double v_x, double v_y, double rot) {
		differentialDrive.arcadeDrive(
				-rot * Controls.getTurnSensitivity(),
				v_x * Controls.getAccelerationSensitivity(),
				true);
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.addDoubleProperty("LeftPosition", () -> motors.getMotor(MotorRole.LeftMaster).getEncoderTicks(), null);
		builder.addDoubleProperty("RightPosition", () -> motors.getMotor(MotorRole.RightMaster).getEncoderTicks(), null);
		builder.addBooleanProperty("CoastMode",
				() -> motors.getIdleMode() == IdleMode.kCoast,
				val -> motors.setIdleMode(val ? IdleMode.kCoast : IdleMode.kBrake));
	}

	// TODO: @Laurin: needs to be implemented correctly!
	@Override
	public void driveToPos(Pose2d pos) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'driveToPos'");
	}
}
