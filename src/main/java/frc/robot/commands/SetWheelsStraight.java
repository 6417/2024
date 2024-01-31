package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetWheelsStraight extends Command {

    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSubsystem.getInstance().stopAllAngleMotors();
    DriveSubsystem.getInstance().setModuleAngle(Constants.OperatorConstants.SWERVE_CALIBRATION_OFFSET,
                                                Constants.OperatorConstants.SWERVE_CALIBRATION_OFFSET,
                                                Constants.OperatorConstants.SWERVE_CALIBRATION_OFFSET,
                                                Constants.OperatorConstants.SWERVE_CALIBRATION_OFFSET);
    addRequirements(DriveSubsystem.getInstance());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.getInstance().stopAllAngleMotors();
    DriveSubsystem.getInstance().setAllAngleEncodersToZero();
  }

  // Returns true when the command should end.

  @Override
  public boolean isFinished() {
    if(Math.abs(DriveSubsystem.getInstance().getLargestAngleError()) < Constants.OperatorConstants.SWERVE_ANGLE_MAX_ERROR) {
      return true;
    } else {
      return false;
    }
  }
}