package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class ClimbDownAction extends Action {
    
    @Override
    public void onStart() {
        Climber.getInstance().wantClimb(false);
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