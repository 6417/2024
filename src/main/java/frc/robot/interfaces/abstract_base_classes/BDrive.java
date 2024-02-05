
package frc.robot.interfaces.abstract_base_classes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.IDrive;

/**
 * BDrive: Abstract base class for all drive subsystems
 * Can't just be a interface because it needs the methods of SybsystemBase
 */
public abstract class BDrive extends SubsystemBase implements IDrive {
}
