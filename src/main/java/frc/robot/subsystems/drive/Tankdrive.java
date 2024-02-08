package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.vision_autonomous.Gyro;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.subsystems.vision_autonomous.Tankdrive_poseestimator;

public class Tankdrive extends DriveBase {

    public TalonFX leftfront = new TalonFX(Constants.Testchassi.idLeftfront);
    public TalonFX rightfront = new TalonFX(Constants.Testchassi.idRigthfront);
    SysIdRoutineLog log;

    public final DutyCycleOut m_leftOut = new DutyCycleOut(0);
    public final DutyCycleOut m_rightOut = new DutyCycleOut(0);

    private StatusSignal rotorpos_left = leftfront.getRotorPosition(); // was ist das
    private StatusSignal rotorpos_rigth = rightfront.getRotorPosition();
    private StatusSignal rotorv_left = leftfront.getVelocity();
    private StatusSignal rotorv_rigth = rightfront.getVelocity();

    public DifferentialDrive differentialDrive = new DifferentialDrive(leftfront::set, rightfront::set);
    public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.5);
    //public DifferentialDriveWheelPositions wheelPositions = new DifferentialDriveWheelPositions(null, null)

    private static Tankdrive instance = null;

    public Tankdrive() {

        leftfront.setInverted(true);

        rightfront.setControl(new Follower(Constants.Testchassi.idRigthback, false));
        leftfront.setControl(new Follower(Constants.Testchassi.idLeftback, false));

        // this.setDefaultCommand(drive_command);
        // drive_command.scedule;

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
                                                leftfront.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(getLeftEncoderPos(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(getWeelSpeeds().leftMetersPerSecond, MetersPerSecond));
                        // Record a frame for the right motors. Since these share an encoder, we
                        // consider the entire group to be one motor.
                        log.motor("drive-right")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                rightfront.get() * RobotController.getBatteryVoltage(), Volts))
                                .linearPosition(m_distance.mut_replace(getRightEndocderPos(), Meters))
                                .linearVelocity(
                                        m_velocity.mut_replace(getWeelSpeeds().rightMetersPerSecond, MetersPerSecond));
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

    public double getLeftEncoderTics() {
        rotorpos_left.refresh();
        return rotorpos_left.getValueAsDouble();
    }

    public double getLeftEncoderPos(){
        return getLeftEncoderTics() * Constants.Testchassi.ticsToMeter;//Constants.Testchassi.Odometry.shaftRotationsToMeters;
    }

    public double getRigthEncoderTics() {
        rotorpos_rigth.refresh();
        return -rotorpos_rigth.getValueAsDouble();
    }

    public double getRightEndocderPos(){
        return getRigthEncoderTics() * Constants.Testchassi.ticsToMeter;//Constants.Testchassi.Odometry.shaftRotationsToMeters;
    }

    public DifferentialDriveWheelSpeeds getWeelSpeeds() {
        //System.out.println(leftfront.getPosition().getUnits());
        rotorv_left.refresh();
        rotorv_rigth.refresh();
        return new DifferentialDriveWheelSpeeds(
                rotorv_left.getValueAsDouble() / 10
                        * Constants.Testchassi.Odometry.shaftRotationsToMeters,
                rotorpos_rigth.getValueAsDouble() / -10
                        * Constants.Testchassi.Odometry.shaftRotationsToMeters);
    }

    public void setVolts(double rigthvolts, double leftvolts) {
        leftfront.setVoltage(leftvolts);
        rightfront.setVoltage(-rigthvolts);
        //leftfront.feed();
        //rightfront.feed();
        // m_drive.feed();
    }

    @Override
    public Pose2d getPos() {
        return Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition();
    }

    public DifferentialDriveWheelPositions getWeelPosition(){
        return new DifferentialDriveWheelPositions(getRightEndocderPos(), getLeftEncoderPos());
    }

    @Override
    public void brake() {
        leftfront.setNeutralMode(NeutralModeValue.Brake);
        rightfront.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void release_brake() {
        rightfront.setNeutralMode(NeutralModeValue.Coast);
        leftfront.setNeutralMode(NeutralModeValue.Coast);
    }

    private double step(double number) {
        // use Math.signum()!
        if (number >= 0) {
            return 1;
        } else {
            return -1;
        }
    }

    // second method to drive robot but does not work
    public void drive2(double v_x, double v_y, double rot) {
        m_rightOut.Output = step(v_x) * (v_x * v_x + rot) / 2;
        m_leftOut.Output = step(v_x) * (v_x * v_x - rot) / 2;
        System.out.println((v_x * v_x - rot) / 2);
        System.out.println(v_x * v_y + rot);

        rightfront.setControl(m_rightOut);
        leftfront.setControl(m_leftOut);
    }

    public void drive(double v_x, double v_y, double rot) {
        differentialDrive.arcadeDrive(
                -rot * Controls.getTurnSensitivity(),
                v_x * Controls.getAccelerationSensitivity(),
                true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("LeftPosition", () -> leftfront.getPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("RightPosition", () -> rightfront.getPosition().getValueAsDouble(), null);
        builder.addBooleanProperty("CoastMode",
                () -> rightfront.getControlMode().getValue() == ControlModeValue.CoastOut,
                val -> 
                    rightfront.setNeutralMode(val? NeutralModeValue.Coast: NeutralModeValue.Brake));
        builder.addDoubleProperty("odometry_x", () -> Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getX(), null);
        builder.addDoubleProperty("odometry_y", () -> Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getY(), null);
        builder.addDoubleProperty("odometry_rot", () -> Tankdrive_odometry.getInstance().m_odometry.getPoseMeters().getRotation().getDegrees(), null);
        builder.addDoubleProperty("gyro_angle", () -> Gyro.getInstance().getRotation2d().getDegrees(), null);
        builder.addDoubleProperty("whee_speed_lef", ()-> getWeelSpeeds().leftMetersPerSecond,null);
    }

    public static Tankdrive getInstance() {
		if (instance == null) {
			if (Constants.Testchassi.driveEnabled) {
				instance = new Tankdrive();
			} else {
				instance = (Tankdrive)new DriveBase();
			}
		}
        return instance;
    }
}
