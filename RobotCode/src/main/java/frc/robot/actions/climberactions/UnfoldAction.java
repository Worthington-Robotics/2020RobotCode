package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class UnfoldAction extends Action {
    @Override
    public void onStart() {
        Climber.getInstance().wantUnfold(true);
        System.out.println("Climb is unfolding.");
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