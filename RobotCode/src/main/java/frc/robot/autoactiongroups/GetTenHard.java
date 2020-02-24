package frc.robot.autoactiongroups;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.ShootAction;

public class GetTenHard extends StateMachineDescriptor {
    public GetTenHard() {
        addParallel(new Action[] {new DummyDrive(), new TurretPIDControl()}, 2500);
        addParallel(new Action[] {new ShootAction(), new TurretPIDControl(), new DummyDrive()}, 8000);
        addParallel(new Action[] {new ShootAction(), new TurretPIDControl()}, 4500);
    }
}