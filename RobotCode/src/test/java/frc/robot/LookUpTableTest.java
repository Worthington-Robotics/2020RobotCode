package frc.robot;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Arrays;

import org.junit.Test;

import frc.robot.subsystems.Shooter;

public class LookUpTableTest {

    @Test
    public void LookUpTableTest(){
        double[] distance = new double[181];
        double[] angleToDistance = new double[181];
        for(int i = 0; i <= 180; i++) {
            distance[i] = (98.5 - Constants.LIMELIGHT_HIGHT) / Math.tan(Math.toRadians((double)i / 2));
            angleToDistance[i] = Shooter.getInstance().limelightRanging((double)i/2);
        }
        System.out.println(Arrays.toString(distance));
        System.out.println(Arrays.toString(angleToDistance));
        assertEquals(distance, angleToDistance);
    }

}