package frc.robot;

//import edu.wpi.first.wpilibj.buttons.JoystickButton;
//import frc.lib.statemachine.Action;
//import frc.robot.actions.*;
import frc.lib.statemachine.Action;
import frc.robot.actions.driveactions.Shift;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.superaction.DeliveryBeltAction;
import frc.robot.actions.superaction.IndexBeltAction;
import frc.robot.actions.superaction.IntakeAction;

public class OI {
    public OI() {
        
        Button indexBeltButton = new JoystickButton(Constants.MASTER, 7);
        Button deliveryBeltButton = new JoystickButton(Constants.MASTER, 8);
        Button intakeButton = new JoystickButton(Constants.MASTER, 9);

        indexBeltButton.whileHeld(Action.toCommand(new IndexBeltAction()));
        deliveryBeltButton.whileHeld(Action.toCommand(new DeliveryBeltAction()));
        intakeButton.whileHeld(Action.toCommand(new IntakeAction()));
    }
}