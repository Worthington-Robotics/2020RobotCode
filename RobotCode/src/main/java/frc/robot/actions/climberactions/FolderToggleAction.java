package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;

public class FolderToggleAction extends Action {
    @Override
    public void onStart() {
        Shooter.getInstance().setTurretCenter(75);
        if (Shooter.getInstance().canUnfold()) {
            Climber.getInstance().setUnfold(true);
        }

    }

    @Override
    public void onLoop() {
        if (Shooter.getInstance().canUnfold()) {
            Shooter.getInstance().setTurretDemand(0);
        }
        Climber.getInstance().setUnfold(true);
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void onStop() {
        Shooter.getInstance().setTurretCenter(75);
        Climber.getInstance().setUnfold(false);
    }
}