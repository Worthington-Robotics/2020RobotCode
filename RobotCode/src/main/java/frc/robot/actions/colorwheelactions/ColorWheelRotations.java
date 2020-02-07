package frc.robot.actions.colorwheelactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.ColorWheel;

public class ColorWheelRotations extends Action {

    @Override
    public void onStart() {
       ColorWheel.getInstance().setColorWheelMode(ColorWheel.colorWheelMode.rotation); 
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