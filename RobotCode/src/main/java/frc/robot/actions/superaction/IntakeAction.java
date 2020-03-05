package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

public class IntakeAction extends Action {
    private Superstructure superstructure;

    public IntakeAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setIntakeDemand(Constants.INTAKE_DEMAND);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return superstructure.getState() == SuperState.FULL_SYSTEM;
    }

    @Override public void onStop() {
        superstructure.setIntakeDemand(Constants.SUPER_DEMAND_STOP);
    }
}
