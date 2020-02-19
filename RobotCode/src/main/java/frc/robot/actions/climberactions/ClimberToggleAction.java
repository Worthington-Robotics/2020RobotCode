package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class ClimberToggleAction extends Action {
    @Override
    public void onStart() {
        if (Climber.getInstance().readyToClimb) {
            Climber.getInstance().setExtend(true);
            //System.out.println("Climb is unfolding.");
        }
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return !Climber.getInstance().readyToDownClimb;
    }

    @Override
    public void onStop() {
        if (Climber.getInstance().readyToDownClimb)
        Climber.getInstance().setExtend(false);
    }
}