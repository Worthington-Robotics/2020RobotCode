package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class LeftClimbAction extends Action {
    @Override public void onStart() {
        Climber.getInstance().setLeftMotorDemand(Constants.CLIMB_DEMAND_LEFT);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        Climber.getInstance().setLeftMotorDemand(Constants.DEMAND_STOP);
    }
}
