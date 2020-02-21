package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;


public class FolderToggleAction extends Action {
    @Override
    public void onStart() {
        if (Climber.getInstance().readyToUnfold()) {
            Shooter.getInstance().setTurretCenter(90);
        }
        
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setTurretDemand(0);
        Climber.getInstance().setUnfold(true);
    }

    @Override
    public boolean isFinished() {
            return !Climber.getInstance().readyToFold();

    }

    @Override
    public void onStop() {
        if (Climber.getInstance().readyToFold()) {
            Shooter.getInstance().setTurretCenter(90);
            Climber.getInstance().setUnfold(false);
        }
    }
}