package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.lib.util.Util;
import frc.robot.Constants;
//import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shooter;

public class AutomaticTurretRPM extends Action {
    private double goalRPM = 0;
    public AutomaticTurretRPM() {
        
    }

    public void onStart() {
        Shooter.getInstance().setRPMOnTarget(false);
        goalRPM = Shooter.getInstance().calcRPM();
        Shooter.getInstance().setFlywheelRPM(goalRPM);
    }

    public void onLoop() {
       
    }

    public boolean isFinished() {
        return Util.epsilonEquals(Shooter.getInstance().getRPMClosedLoopError(), 0, Constants.RPM_ACCEPTIBLE_ERROR);
    }

    public void onStop() {
        Shooter.getInstance().setRPMOnTarget(true);
    }
}
