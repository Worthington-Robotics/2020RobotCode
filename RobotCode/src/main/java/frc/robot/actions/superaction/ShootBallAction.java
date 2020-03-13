package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class ShootBallAction extends Action {
    @Override public void onStart() {}

    @Override public void onLoop() {
        Superstructure.getInstance().shootBall();
    }

    @Override public boolean isFinished() {
        return true;
    }

    @Override public void onStop() {}
}
