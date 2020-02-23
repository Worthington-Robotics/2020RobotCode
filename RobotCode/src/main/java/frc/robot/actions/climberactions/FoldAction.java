package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class FoldAction extends Action {
    private boolean done;
    @Override
    public void onStart() {
        Climber.getInstance().wantUnfold(false);
        done = true;
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void onStop() {
        
    }
}