package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class PulseIndexBeltAction extends Action {
    @Override
    public void onStart() {
        //System.out.println("Action Starting");
    }

    @Override
    public void onLoop() {
        Superstructure.getInstance().setIndexBeltDemand(
                Superstructure.getInstance().pulse(
                    Superstructure.getInstance().getIndexerDemand(), 1, 2));
        //System.out.println("Looping");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Superstructure.getInstance().setIndexBeltDemand(0);
    }

}