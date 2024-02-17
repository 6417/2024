package frc.robot;

import java.util.List;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.fridowpi.joystick.JoystickHandler;
import frc.fridowpi.joystick.joysticks.Logitech;
import frc.fridowpi.sensors.FridoNavx;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.vision_autonomous.Gyro;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;
import frc.robot.subsystems.drive.Controls;
import frc.robot.subsystems.drive.Tankdrive;
import frc.robot.subsystems.drive.getAutonomousTrajectory;
import frc.robot.subsystems.drive.getRamsetCommand;
import frc.robot.subsystems.drive.swerve.SwerveDrive;
import frc.robot.subsystems.drive.swerve.SwerveDriveBase;
import frc.robot.subsystems.vision_autonomous.Gyro;
import frc.robot.subsystems.vision_autonomous.Tankdrive_odometry;

public class Robot extends TimedRobot {

    // Aliases for often used singleton instances
    SwerveDriveBase drive = SwerveDrive.getInstance();
    // ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    ShuffleboardTab tab;

    @Override
    public void robotInit() {

		FridoNavx.setup(SPI.Port.kMXP);

        JoystickHandler.getInstance().setupJoysticks(List.of(Constants.Joystick.id));
        JoystickHandler.getInstance().init();
        JoystickHandler.getInstance().bind(SwerveDrive.getInstance());
        JoystickHandler.getInstance().init();

        SwerveDrive.getInstance();


        // Add subsystems
        Shuffleboard.getTab("Drive").add(drive);
        // Shuffleboard.getTab("Shooter").add(shooter);

        // Shuffleboard.getTab("Controls").add(Controls.getInstance()); // wtf nr. 2
        // Shuffleboard.getTab("Debug").add(drive.getDefaultCommand());
        Shuffleboard.getTab("Debug").add(CommandScheduler.getInstance());

        // SignalLogger.setPath("/media/sda1");
        SignalLogger.setPath("/home/lvuser/logs");

        tab = Shuffleboard.getTab("robotpos");
        
        JoystickButton resetButton = new JoystickButton(Controls.joystick, Logitech.b.getButtonId());
        resetButton.onTrue(new InstantCommand(() -> {
            Gyro.getInstance().reset();
            Tankdrive_odometry.getInstance().reset_odometry();
        }));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        Tankdrive_odometry.getInstance().update_robot_pose();
    }

    @Override
    public void teleopInit() {
        SignalLogger.setPath("test");
        // shooter.setMotorSpeed(0);
    }

    // AutoCommand aComand = null;
    Command auto_command = null;
    Servo exampleServo = new Servo(0);
    //PWM pwm = new PWM(0);

    @Override
    public void teleopPeriodic() {

        //Tankdrive.getInstance().differentialDrive.feed();
        //exampleServo.setBoundsMicroseconds(2150, 1501, 1500, 1499, 850);
        //exampleServo.setPulseTimeMicroseconds((int) (000*Controls.joystick.getRightX()));
        //exampleServo.set(Controls.joystick.getRightX());

        Tankdrive.getInstance().differentialDrive.feed();
        
        if (Controls.joystick.getYButtonPressed()) {
            System.out.println("start command");
            auto_command = getRamsetCommand.getInstance().start_command();
        }
        
        
        if (Controls.joystick.getLeftBumperPressed()) {
            Tankdrive.getInstance().brake();
        }
        if (Controls.joystick.getRightBumperPressed()) {
            Tankdrive.getInstance().release_brake();
        }
        
        //bindButtonsForSysid();
    }

    /*
     * if (Constants.Sysid.isTuning) {
     * bindButtonsForSysid();
     * } else {
     * bindButtonsDefault();
     * shooter.run(true);
     * }
     * 
     * // Brake mode
     * if (Controls.joystick.getLeftBumperPressed()) {
     * drive.release_brake();
     * }
     * if (Controls.joystick.getRightBumperPressed()) {
     * drive.brake();
     * }
     * 
     * // System.out.println(Tankdrive_poseestimator.getInstance().m_poseEstimator.
     * getEstimatedPosition());
     * }
     * 
     * private void bindButtonsDefault() {
     * // Shooter
     * if (Controls.joystick.getPOV() == 0) {
     * shooter.changeMotorSpeed(1);
     * } else if (Controls.joystick.getPOV() == 180) {
     * shooter.changeMotorSpeed(-1);
     * }
     * if (Controls.joystick.getYButtonPressed()) {
     * shooter.setMotorSpeed(Constants.Shooter.OptimalSpeakerSpeed);
     * } else if (Controls.joystick.getXButtonPressed()) {
     * shooter.setMotorSpeed(Constants.Shooter.OptimalAmpSpeed);
     * } else if (Controls.joystick.getBButtonPressed()) {
     * shooter.setMotorSpeed(0.0);
     * }
     * }
     */
}
