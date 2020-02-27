package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class SuperstructureDisable extends Action {

    @Override public void onStart() {
        Superstructure.getInstance().disable();
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return true;
    }

    @Override public void onStop() {
    }
}
