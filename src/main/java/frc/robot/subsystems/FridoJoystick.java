package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.fridowpi.joystick.WPIJoystick;

public class FridoJoystick {

    private static FridoJoystick instance;

    private Typ type_of_controler = Typ.xBoxController;

    public Joystick joystick = new Joystick(0);

    enum Typ{
        xBoxController,
        Joystick,
    }
    
    private FridoJoystick(){
        
    }
    

    public static FridoJoystick getInstance(){
        if (instance == null){
            instance = new FridoJoystick();
        }
        return instance;
    }
}
