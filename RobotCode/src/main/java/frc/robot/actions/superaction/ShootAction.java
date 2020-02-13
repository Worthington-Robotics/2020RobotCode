package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class ShootAction extends Action {
    private Superstructure superstructure;

    public ShootAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.shootBall();
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {}
}
