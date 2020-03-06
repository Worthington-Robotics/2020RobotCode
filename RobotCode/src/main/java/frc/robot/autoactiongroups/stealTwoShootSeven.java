package frc.robot.autoactiongroups;

import java.util.ArrayList;
import frc.lib.control.Path;
import frc.lib.control.Path.Waypoint;
import frc.lib.geometry.Translation2d;
import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.driveactions.DriveTra;
import frc.robot.actions.shooteraction.TurretPIDControl;
import frc.robot.actions.superaction.IntakeAction;
import frc.robot.actions.superaction.ShootAction;

public class stealTwoShootSeven extends StateMachineDescriptor {
    public stealTwoShootSeven() {
        ArrayList<Waypoint> waypoints = new ArrayList<>();
        ArrayList<Waypoint> waypoints2 = new ArrayList<>();
        ArrayList<Waypoint> waypoints3 = new ArrayList<>();
        ArrayList<Waypoint> waypoints4 = new ArrayList<>();
        waypoints.add(new Waypoint(new Translation2d(0,0), 72));
        waypoints.add(new Waypoint(new Translation2d(125, 0), 72));
        Path stealingPath1 = new Path(waypoints);
        waypoints2.add(new Waypoint(new Translation2d(125, 0), 72));
        waypoints2.add(new Waypoint(new Translation2d(90, 130), 72));
        waypoints2.add(new Waypoint(new Translation2d(85, 125), 50));
        Path stealingPath2 = new Path(waypoints2);
        waypoints3.add(new Waypoint(new Translation2d(85, 125), 50));
        waypoints3.add(new Waypoint(new Translation2d(92, 125), 72));
        Path stealingPath3 = new Path(waypoints3);
        waypoints4.add(new Waypoint(new Translation2d(92, 125), 72));
        waypoints4.add(new Waypoint(new Translation2d(45, 130), 72));
        Path stealingPath4 = new Path(waypoints4);

        //Actual Auto
        addParallel(new Action[]{new DriveTra(stealingPath1, false), new IntakeAction(), new TurretPIDControl()}, 5000);
        addParallel(new Action[]{new DriveTra(stealingPath2, true), new ShootAction(), new TurretPIDControl()}, 8000);
        addParallel(new Action[]{new DriveTra(stealingPath3, false), new IntakeAction(), new TurretPIDControl()}, 3000);
        addParallel(new Action[]{new DriveTra(stealingPath4, true), new ShootAction(), new TurretPIDControl()}, 3000);
    }
}