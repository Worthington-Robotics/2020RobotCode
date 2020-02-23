package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;

public class FolderToggleAction extends Action {

    @Override
    public void onStart() {
        Climber.getInstance().wantUnfold(true);
    }

    @Override
    public void onLoop() {
        
    }

    @Override
    public boolean isFinished() {
        return !Climber.getInstance().canUnfold();

    }

    @Override
    public void onStop() {
        Climber.getInstance().wantUnfold(false);      
    }
}