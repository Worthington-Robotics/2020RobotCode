package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootAction;

public class get10hard extends StateMachineDescriptor {
    public get10hard() {
        addSequential(new TurretPIDControl(), 1000);
        addParallel(new Action[] {new DummyDrive(), new TurretPIDControl()}, 2000);
        addParallel(new Action[] {new ShootAction(), new TurretPIDControl(), new DummyDrive()}, 7000);
        addParallel(new Action[] {new ShootAction(), new TurretPIDControl()}, 5000);
    }
}