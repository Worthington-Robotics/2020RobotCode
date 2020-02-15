package frc.robot.actions.superaction;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Superstructure;

public class PulseIndexBeltAction extends Action {
    private double demand;
    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {
        Superstructure.getInstance().pulse(demand, 1, 5000);
        Superstructure.getInstance().setIntakeDemand(demand);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {

    }

}