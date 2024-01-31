// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;
import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CalibrateSwerveMotorsToSensor;
import edu.wpi.first.wpilibj.Joystick;

public class DriveSubsystem extends SubsystemBase {
  private Joystick contrJoystick;
  private WPI_TalonSRX motorFrontRight,motorFrontRightRotate, motorFrontLeft, motorFrontLeftRotate, motorRearRight,motorRearRightRotate, motorRearLeft,motorRearLeftRotate;

  public DriveSubsystem() {
    motorFrontRight = new WPI_TalonSRX(Constants.OperatorConstants.FRONT_RIGHT);
    motorFrontRightRotate = new WPI_TalonSRX(Constants.OperatorConstants.FRONT_RIGHT_ROTATE);
    motorFrontLeft = new WPI_TalonSRX(Constants.OperatorConstants.FRONT_LEFT);
    motorFrontLeftRotate = new WPI_TalonSRX(Constants.OperatorConstants.FRONT_LEFT_ROTATE);
    motorRearRight = new WPI_TalonSRX(Constants.OperatorConstants.REAR_RIGHT);
    motorRearRightRotate = new WPI_TalonSRX(Constants.OperatorConstants.REAR_RIGHT_ROTATE);
    motorRearLeft = new WPI_TalonSRX(Constants.OperatorConstants.REAR_LEFT);
    motorRearLeftRotate = new WPI_TalonSRX(Constants.OperatorConstants.REAR_LEFT_ROTATE);

    for (WPI_TalonSRX motor:this.getRotationMotors()) {
      configMotor(motor);
    }

    contrJoystick = new Joystick(0);
    motorFrontRight.setInverted(true);
    motorRearLeftRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motorRearRightRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motorFrontLeftRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    motorFrontRightRotate.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    SmartDashboard.putData(this);
  }

  private static DriveSubsystem instance = null;
  public static DriveSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveSubsystem();
    }
    return instance;
  }


  private void configMotor(WPI_TalonSRX motor) {
    motor.configFactoryDefault();
    motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.config_kD(0, 2);
    motor.config_kP(0, 0.03);
  }

  private WPI_TalonSRX[] getRotationMotors() {
    return new WPI_TalonSRX[] {motorFrontLeftRotate, motorFrontRightRotate, motorRearLeftRotate, motorRearRightRotate};
  }

  public void enableRotationLimitSwitch(boolean enabled) {
    for(WPI_TalonSRX rotateMotor:this.getRotationMotors()) {
      if (enabled == true) {
        rotateMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
      } else {
        rotateMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled);
      }
    }
  }

  public void setModuleAngle(double frontLeft, double frontRight, double backLeft, double backRight) {
    motorFrontLeftRotate.set(TalonSRXControlMode.Position, convertAngleToEncoderTicks(frontLeft));
    motorFrontRightRotate.set(TalonSRXControlMode.Position, convertAngleToEncoderTicks(frontRight));
    motorRearLeftRotate.set(TalonSRXControlMode.Position, convertAngleToEncoderTicks(backLeft));
    motorRearRightRotate.set(TalonSRXControlMode.Position, convertAngleToEncoderTicks(backRight));
  }

  public void setMotorSpeed(double frontLeft, double frontRight, double backLeft, double backRight) {
    motorFrontLeft.set(TalonSRXControlMode.PercentOutput, frontLeft);
    motorFrontRight.set(TalonSRXControlMode.PercentOutput, frontRight);
    motorRearLeft.set(TalonSRXControlMode.PercentOutput, backLeft);
    motorRearRight.set(TalonSRXControlMode.PercentOutput, backRight);
  }

  public double convertAngleToEncoderTicks(double angle) {
    return Constants.OperatorConstants.SWERVE_TICK_COUNT/360 * angle;
  }

  public boolean allRotationLimitswitchesActive() {
    for(TalonSRX motor: this.getRotationMotors()) {
      if(motor.isFwdLimitSwitchClosed() == 0) {
        return false;
      }
    }
    return true;
  }

  public void stopAllAngleMotors() {
    motorFrontLeftRotate.stopMotor();
    motorFrontRightRotate.stopMotor();
    motorRearLeftRotate.stopMotor();
    motorRearRightRotate.stopMotor();
  }

  public void setAllAngleEncodersToZero () {
    motorFrontLeftRotate.setSelectedSensorPosition(0);
    motorFrontRightRotate.setSelectedSensorPosition(0);
    motorRearLeftRotate.setSelectedSensorPosition(0);
    motorRearRightRotate.setSelectedSensorPosition(0);
  }

  public double getLargestAngleError() {
    Collection<Double> motorErrors = Arrays.asList(motorFrontLeftRotate.getClosedLoopError(), motorFrontRightRotate.getClosedLoopError(), motorRearLeftRotate.getClosedLoopError(), motorRearRightRotate.getClosedLoopError());
    return Collections.max(motorErrors);
  }

  public static double calculateAngle(double x, double y) {
    return Math.toDegrees(Math.atan2(x, y));
}

  @Override
  public void periodic() {
    setMotorSpeed(0.2, 0.2, 0.2, 0.2);
    setModuleAngle(calculateAngle(contrJoystick.getY(),contrJoystick.getX()),calculateAngle(contrJoystick.getY(),contrJoystick.getX()),calculateAngle(contrJoystick.getY(),contrJoystick.getX()),calculateAngle(contrJoystick.getY(),contrJoystick.getX()));
    System.out.println(calculateAngle(contrJoystick.getX(), -contrJoystick.getY()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addIntegerProperty("Rotate-Rear-Left-HALL-Sensor", motorRearLeftRotate::isFwdLimitSwitchClosed, null);
      builder.addIntegerProperty("Rotate-Front-Left-HALL-Sensor", motorFrontLeftRotate::isFwdLimitSwitchClosed, null);
      builder.addIntegerProperty("Rotate-Rear-Right-HALL-Sensor", motorRearRightRotate::isFwdLimitSwitchClosed, null);
      builder.addIntegerProperty("Rotate-Front-Right-HALL-Sensor", motorFrontRightRotate::isFwdLimitSwitchClosed, null);
      builder.addDoubleProperty("Rotate-Rear-Left-Position", motorRearLeftRotate::getSelectedSensorPosition, null);
      builder.addDoubleProperty("Rotate-Front-Left-Position", motorFrontLeftRotate::getSelectedSensorPosition, null);
      builder.addDoubleProperty("Rotate-Rear-Right-Position", motorRearRightRotate::getSelectedSensorPosition, null);
      builder.addDoubleProperty("Rotate-Front-Right-Position", motorFrontRightRotate::getSelectedSensorPosition, null);
     
      super.initSendable(builder);
  }
}
