package frc.robot.actions.climberactions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class RightClimbAction extends Action {
    @Override public void onStart() {
        Climber.getInstance().setRightMotorDemand(Constants.CLIMB_DEMAND_RIGHT);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        Climber.getInstance().setRightMotorDemand(Constants.DEMAND_STOP);
    }
}
