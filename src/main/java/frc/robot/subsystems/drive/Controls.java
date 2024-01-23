package frc.robot.subsystems.drive;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;

/** Holds the data concerning input, which should be available
 * either to the entire program or get exported to the suffleboard */
public class Controls implements Sendable {
    private static Controls instance = new Controls();
    
    final public static XboxController joystick = new XboxController(0);
    private static double turnSensitivity = 0.6;
    private static double accelerationSensitivity = 0.6;

    // Initialization
    private Controls() {}
    public static Controls getInstance() {
        return instance;
    }

    // Getters and setters
    public static double getAccelerationSensitivity() {
        return accelerationSensitivity;
    }

    public static double getTurnSensitivity() {
        return turnSensitivity;
    }
    
    // Shuffleboard
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        
        builder.addDoubleProperty("turnSensitivity", () -> turnSensitivity,
                val -> turnSensitivity = val);
        builder.addDoubleProperty("accelerationSensitivity", () -> accelerationSensitivity,
                val -> accelerationSensitivity = val);

        // Joystick controls
        builder.addDoubleProperty("joystickLeftX", Controls.joystick::getLeftX, null);
        builder.addDoubleProperty("joystickLeftY", Controls.joystick::getLeftY, null);
        builder.addDoubleProperty("joystickRightX", Controls.joystick::getRightX, null);
        builder.addDoubleProperty("joystickRightY", Controls.joystick::getRightY, null); // Not working: wtf?!?
    }

}
