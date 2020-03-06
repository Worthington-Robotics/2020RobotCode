package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DummyDrive;

public class getTenEasy extends StateMachineDescriptor {
    public getTenEasy() {
        addSequential(new DummyDrive(true), 2000);
    }
}