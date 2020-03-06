package frc.robot.autoactiongroups;

import java.util.ArrayList;

import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAction;

public class stealingTwoFromEnemy extends StateMachineDescriptor {
    public stealingTwoFromEnemy() {
        //Waypoint Setup
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        ArrayList<Waypoint> waypoints2 = new ArrayList<>();
        waypoints.add(new Waypoint(new Translation2d(0,0), 72));
        waypoints.add(new Waypoint(new Translation2d(125, 0), 72));
        Path stealingPath1 = new Path(waypoints);
        waypoints2.add(new Waypoint(new Translation2d(125, 0), 72));
        waypoints2.add(new Waypoint(new Translation2d(90, 130), 72));
        Path stealingPath2 = new Path(waypoints2);
        //Actual Auto
        addParallel(new Action[]{new DriveTra(stealingPath1, false), new IntakeAction()}, 5000);
        addSequential(new DriveTra(stealingPath2, true), 10000);
        addSequential(new ShootAction(), 5000);
    }
}