package frc.robot.actions.buttonactions;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.statemachine.Action;

public class ModAction extends Action {

    Action main, function;
    JoystickButton button;
    boolean isMod;

    public ModAction(Action main, Action function, JoystickButton button) {
        this.main = main;
        this.function = function;
        this.button = button;
    }

    @Override
    public void onStart() {
        isMod = button.get();
        System.out.println("Activated");
        if (!isMod) {
            System.out.println("Normal");
            main.onStart();
        } else {
            System.out.println("Alt");
            function.onStart();
        }
    }

    @Override
    public void onLoop() {
        if (!isMod) {
            main.onLoop();
        } else {
            function.onLoop();
        }
    }

    @Override
    public boolean isFinished() {
        if (!isMod) {
            return main.isFinished();
        } else {
            return function.isFinished();
        }
    }

    @Override
    public void onStop() {
        if (!isMod) {
            main.onStop();
        } else {
            function.onStop();
        }
    }
}
