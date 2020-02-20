package frc.robot.autoactiongroups;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;

public class GetTenFEAT extends StateMachineDescriptor{
    public GetTenFEAT() {
        addSequential(new DriveTra(new Path(tenFeet()), false), 10000);
    }

    private List<Path.Waypoint> tenFeet() {
        return Arrays.asList(
                new Waypoint(new Translation2d(0,0), 0),
                new Waypoint(new Translation2d(0,120), 0));
    }
}