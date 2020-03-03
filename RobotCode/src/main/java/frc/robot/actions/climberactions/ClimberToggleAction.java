package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;


public class ClimberToggleAction extends Action {
    @Override
    public void onStart() {
        Climber.getInstance().wantClimb(true);
    }

    @Override
    public void onLoop() {

    }

    @Override
    public boolean isFinished() {
        return !Climber.getInstance().canClimb();
    }

    @Override
    public void onStop() {
        Climber.getInstance().wantClimb(false);
    }
}