package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class IntakeAction extends Action {
    private Superstructure superstructure;

    public IntakeAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setIntaking(true);
    }

    @Override public void onLoop() {
        superstructure.setIntaking(true);}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setIntaking(false);
    }
}
