package frc.robot.autoactiongroups;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.*;

import java.util.ArrayList;

import frc.lib.geometry.Translation2d;
import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;

public class SkewLeft extends StateMachineDescriptor {
    public SkewLeft() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Waypoint(new Translation2d(0,0), 72));
        waypoints.add(new Waypoint(new Translation2d(60, 0), 72));
        waypoints.add(new Waypoint(new Translation2d(120, 24), 72));
        Path left = new Path(waypoints);
        addSequential(new FollowPath(left), 10000);
    }
}