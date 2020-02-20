package frc.robot.autoactiongroups;

import java.util.ArrayList;
import java.util.List;

import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.subsystems.Drive;

public class GetTenFEAT extends StateMachineDescriptor{
    private List<Path.Waypoint> GetTenFeet() {
        List<Path.Waypoint> mPath = new ArrayList<>();
        mPath.add(new Waypoint(new Translation2d(0,0), 0));
        mPath.add(new Waypoint(new Translation2d(0,120), 0));
        return mPath;
    }
    public GetTenFEAT() {
        
        addSequential(new DriveTra(new Path(GetTenFeet()), false), 10000);
    }
}