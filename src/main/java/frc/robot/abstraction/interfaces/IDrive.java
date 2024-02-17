package frc.robot.abstraction.interfaces;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.fridowpi.joystick.JoystickBindable;
import frc.fridowpi.module.IModule;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;

public interface IDrive extends IModule, Sendable, JoystickBindable {

    // Steering functions
    public void drive(double v_x, double v_y, double rot);

    public void setVolts(double leftvolts, double rigthvolts);

    public void driveToPos(Pose2d pos);

    // Braking Mode
    public void setIdleMode(IdleMode mode);

    // Sysid tuning (not really necessary)
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction);

    public Command sysIdDynamic(SysIdRoutine.Direction direction);

    // Getters
    public Pose2d getPos();

    public double getLeftEncoderPos();

    public double getRightEncoderPos();


    // Couldn't (yet) generalize these:
	//
	// Tankdrive only
    public Optional<DifferentialDriveKinematics> getDifferentialKinematics();
	public Optional<DifferentialDriveWheelSpeeds> getDifferentialWheelSpeeds();

	// Swerve only
    public Optional<SwerveDriveKinematics> getSwerveKinematics();

    // Abstraction stuff
    public boolean isSwerve();

    // Must be sendable
    public void initSendable(SendableBuilder builder);
}