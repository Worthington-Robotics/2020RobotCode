package frc.robot.actions.colorwheelactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.ColorWheel;

public class ColorWheelPosition extends Action {

    @Override
    public void onStart() {
        ColorWheel.getInstance().setColorWheelMode(ColorWheel.colorWheelMode.position); 
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {

    }

}