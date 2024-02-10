package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.fridowpi.joystick.IJoystick;
import frc.fridowpi.joystick.JoystickHandler;

/** Holds the data concerning input, which should be available
 * either to the entire program or get exported to the suffleboard */
public class Controls implements Sendable {
    private static Controls instance = new Controls();
    
    
    private static double turnSensitivity = 0.6;
    private static double accelerationSensitivity = 0.6;
    public static IJoystick joystick = JoystickHandler.getInstance().getJoystick(Constants.Joystick.primaryJoystickId);

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
    }

}
