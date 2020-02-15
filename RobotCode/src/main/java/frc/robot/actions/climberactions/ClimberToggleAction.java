package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class ClimberToggleAction extends Action {
    @Override
    public void onStart() {
        Climber.getInstance().setExtend(true);
        //System.out.println("Climb is unfolding.");
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
       return false;
    }

    @Override
    public void onStop() {
        Climber.getInstance().setExtend(false);
    }
}