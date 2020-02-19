package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;

public class GetTenEasy extends StateMachineDescriptor {
    public GetTenEasy() {
        addSequential(new DummyDrive(), 1000);
    }
}