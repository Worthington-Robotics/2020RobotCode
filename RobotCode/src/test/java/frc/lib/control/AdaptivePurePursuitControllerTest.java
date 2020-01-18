package frc.lib.control;

import static org.junit.Assert.assertTrue;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.lib.geometry.Twist2d;

public class AdaptivePurePursuitControllerTest {

    @Test
    public void testConvergence(){
        
        //Test parameters
        Pose2d end = new Pose2d(100, 0, Rotation2d.fromDegrees(0));

        //slows the convergence down because the pose calculation is not 
        //quite a perfect update from the pose estimator. This model ignores
        //any effects from inertia
        double friction_factor = 0.008; 

        //turn factor induces drag on the controller to simulate a drivetrain
        // that favors turning one direction (right). > 1 will simulate an oversteering
        // while < 1 will simulate a drivetrain that is less reactive
        double turn_factor = 1.05;

        double lookahead = 24.0; //in
        double max_accel = 48.0; //in/s^2
        double nominal_dt = 0.010; //s
        boolean reversed = false;
        double path_completion_tolerance = 1.0; //in


        List<Path.Waypoint> waypoints = new ArrayList<>();
        waypoints.add(new Path.Waypoint(new Translation2d(0,0), 40.0));
        waypoints.add(new Path.Waypoint(end.getTranslation(), 40.0));
        Path path = new Path(waypoints);

        //create the conrtoller object 
        AdaptivePurePursuitController controller =
         new AdaptivePurePursuitController(lookahead, max_accel,
          nominal_dt, path, reversed, path_completion_tolerance);
        Pose2d pose = new Pose2d();

        final DecimalFormat fmt = new DecimalFormat("#0.000");

        for(double t = 0.0; t < 5.0; t += nominal_dt){
            Twist2d update = controller.update(pose, t);

            //scale the update of the pose by by the friction and turn factors
            update = new Twist2d(update.dx * friction_factor, update.dy * 
             friction_factor, update.dtheta * turn_factor);

            //update the pose with the specified transform
            pose = pose.transformBy(Pose2d.exp(update));

            //print the data to the console to see what is going on
            System.out.println(fmt.format(t) + ", " + pose.toString() + ", " + update.toString());
        }
        
        //give the test a conditional to fail on should something in the controller change
        assertTrue(pose.epsilonEquals(end, path_completion_tolerance));
    }

}