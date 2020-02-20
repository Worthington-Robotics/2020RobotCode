package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class PulseIndexBeltAction extends Action {
    Superstructure superstructure;

    public PulseIndexBeltAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() { }

    @Override
    public void onLoop() {
        superstructure.pulseIndexDemand();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        superstructure.setIndexBeltDemand(0);
    }

}