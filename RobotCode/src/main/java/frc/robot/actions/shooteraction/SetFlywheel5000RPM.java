package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class SetFlywheel5000RPM extends Action {

    

    @Override
    public void onStart() {
        double RPM = Shooter.getInstance().RPMToTicksPer100ms(5000);
        Shooter.getInstance().setFlywheelRPM(RPM);
    }

    @Override
    public void onLoop() {
        double RPM = Shooter.getInstance().RPMToTicksPer100ms(5000);
        Shooter.getInstance().setFlywheelRPM(RPM);

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setFlywheelRPM(0);

    }

}