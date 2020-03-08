package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class ShootAllAction extends Action {
    @Override public void onStart() {
        Superstructure.getInstance().setShooting(true);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return Superstructure.getInstance().isSystemEmpty();
    }

    @Override public void onStop() {
        Superstructure.getInstance().setShooting(false);
    }
}
