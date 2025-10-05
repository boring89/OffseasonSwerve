package frc.robot.subsystems.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller extends XboxController {
    
    public Controller(int port) {
        super(port);
    }

    public Trigger zeroHeading() {
        return new Trigger(this::getAButton);
    }
}
