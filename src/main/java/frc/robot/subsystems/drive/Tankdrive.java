package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Tankdrive_poseestimator;

public class Tankdrive extends DriveBase {

  TalonFX leftfront = new TalonFX(Constants.Testchassi.idLeftfront);
  TalonFX rightfront = new TalonFX(Constants.Testchassi.idRigthfront);

  private final DutyCycleOut m_leftOut = new DutyCycleOut(0);
  private final DutyCycleOut m_rightOut = new DutyCycleOut(0);

  private StatusSignal rotorpos_left = leftfront.getRotorPosition();
  private StatusSignal rotorpos_rigth = rightfront.getRotorPosition();
  private StatusSignal rotorv_left = leftfront.getRotorVelocity();
  private StatusSignal rotorv_rigth = rightfront.getRotorVelocity();

  private DifferentialDrive differentialDrive = new DifferentialDrive(leftfront::set, rightfront::set);
  public DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(0.7);

  private static Tankdrive instance;

  public Tankdrive() {

    rightfront.setInverted(true);

    rightfront.setControl(new Follower(Constants.Testchassi.idRigthback, false));
    leftfront.setControl(new Follower(Constants.Testchassi.idLeftback, false));

    //this.setDefaultCommand(drive_command);
    //drive_command.scedule;

    setDefaultCommand(new DriveCommand(this));
  }

  // Mutable holders for unit-safe values, persisted to avoid reallocation.
  final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(t -> setVolts(t.baseUnitMagnitude(), t.baseUnitMagnitude()),

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
            // consider
            // the entire group to be one motor.
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightfront.get() * RobotController.getBatteryVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(getRigthEncoderPos(), Meters))
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
  public void periodic() { }

  public double getLeftEncoderPos() {
    rotorpos_left.refresh();
    return rotorpos_left.getValueAsDouble();
  }

  public double getRigthEncoderPos() {
    rotorpos_rigth.refresh();
    return rotorpos_rigth.getValueAsDouble();
  }

  public DifferentialDriveWheelSpeeds getWeelSpeeds() {
    rotorv_left.refresh();
    rotorv_rigth.refresh();
    return new DifferentialDriveWheelSpeeds(
        rotorv_left.getValueAsDouble() * 10
            * Constants.Testchassi.Odometry.encoderToMetersConversion,
        rotorpos_rigth.getValueAsDouble() * -10
            * Constants.Testchassi.Odometry.encoderToMetersConversion);
  }

  public void setVolts(double leftvolts, double rigthvolts){
    leftfront.setVoltage(leftvolts);
    rightfront.setVoltage(rigthvolts);
    //m_drive.feed();
  }

  @Override
  public Pose2d getPos() {
    return Tankdrive_poseestimator.getInstance().m_poseEstimator.getEstimatedPosition();
  }

  private double step(double number) {
    // use Math.signum()!
    if (number >= 0) {
      return 1;
    } else {
      return -1;
    }
  }

  //second method to drive robot but does not work
  public void drive2(double v_x, double v_y, double rot){
    m_rightOut.Output = step(v_x) * (v_x*v_x + rot)/2;
    m_leftOut.Output = step(v_x) * (v_x*v_x - rot)/2;
    System.out.println((v_x*v_x - rot)/2);
    System.out.println(v_x*v_y + rot);

    rightfront.setControl(m_rightOut);
    leftfront.setControl(m_leftOut);
  }

  public void drive(double v_x, double v_y, double rot){
    //leftback.set(v_x);
    //rightback.set(v_x);
    System.out.println(rot);
    differentialDrive.arcadeDrive(v_x, rot);
  }

  public static Tankdrive getInstance() {
    if (instance == null) {
      instance = new Tankdrive();
    }
    return instance;
  }
}
