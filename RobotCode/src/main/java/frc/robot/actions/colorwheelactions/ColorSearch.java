/*package frc.robot.actions.colorwheelactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.ColorWheel;

public class ColorSearch extends Action {

    @Override
    public void onStart() {
        ColorWheel.getInstance().setColorMotorPidOn(true);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return ColorWheel.getInstance().checkColor();
    }

    @Override
    public void onStop() {
        ColorWheel.getInstance().setColorMotorPidOn(false);
    }
}
*/