package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.climberactions.*;
import frc.robot.actions.shooteraction.*;
import frc.robot.actions.superaction.*;

public class SystemsCheck extends StateMachineDescriptor {
    public SystemsCheck() {
        addSequential(new UnfoldAction(), 1000);
        addParallel(new Action[]{new ClimberToggleAction(), new ArmAction()}, 2000);
        addSequential(new FoldAction(), 1000);
        addParallel(new Action[]{
            new IntakeAction(), 
            new DeliveryBeltAction(), 
            new DeliveryWheelAction()
        }, 1000);
        addSequential(new SetFlywheel5000RPM(), 2000);
        addSequential(new Recenter(0), 1000);
        addSequential(new Recenter(-45), 1000);
        addSequential(new Recenter(45), 1000);
    }
}