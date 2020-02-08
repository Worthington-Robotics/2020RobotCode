package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.ColorWheel;

public class colorWheelManual extends Action {
    private boolean CCW = false;

    public colorWheelManual(boolean CCW)
    {
        this.CCW = CCW;
    }

    @Override
    public void onStart() {
        ColorWheel.getInstance().setEnabled(true);
        ColorWheel.getInstance().setCCW(CCW);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        ColorWheel.getInstance().setEnabled(false);
    }
}
