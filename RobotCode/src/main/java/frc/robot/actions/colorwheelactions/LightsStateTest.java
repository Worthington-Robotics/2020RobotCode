package frc.robot.actions.colorwheelactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Lights;

public class LightsStateTest extends Action {
    private int state = 0;

    public LightsStateTest(int state)
    {
        this.state = state;
    }

    @Override
    public void onStart() {
        Lights.getInstance().testLights(state);
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
