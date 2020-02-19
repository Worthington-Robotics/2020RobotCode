package frc.lib.util;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class HIDHelperTest{

    @Test
    public void testManipulation(){

        double res = HIDHelper.evalDeadBand(-1, .1, 2);
        assertEquals(-1, res, 0.01);

        res = HIDHelper.evalDeadBand(1, .1, 2);
        assertEquals(1, res, 0.01);

        res = HIDHelper.evalDeadBand(-0.5, .1, 2);
        assertEquals(-0.25, res, 0.01);

        res = HIDHelper.evalDeadBand(0.5, .1, 2);
        assertEquals(0.25, res, 0.01);

        res = HIDHelper.evalDeadBand(-0.01, .1, 2);
        assertEquals(-0, res, 0.01);

        res = HIDHelper.evalDeadBand(0.01, .1, 2);
        assertEquals(0, res, 0.01);

    } 

    @Test
    public void testMap(){
        double map = HIDHelper.getAxisMapped(-1, 0, 1);
        assertEquals(0, map, 0.01);
        
        map = HIDHelper.getAxisMapped(0, 0, 1);
        assertEquals(0.5, map, 0.01);

        map = HIDHelper.getAxisMapped(1, 0, 1);
        assertEquals(1, map, 0.01);

        map = HIDHelper.getAxisMapped(-1, 0, 100);
        assertEquals(0, map, 0.01);

        map = HIDHelper.getAxisMapped(0, 0, 100);
        assertEquals(50, map, 0.01);

        map = HIDHelper.getAxisMapped(1, 0, 100);
        assertEquals(100, map, 0.01);

        map = HIDHelper.getAxisMapped(-1, -100, 0);
        assertEquals(-100, map, 0.01);

        map = HIDHelper.getAxisMapped(0, -100, 0);
        assertEquals(-50, map, 0.01);

        map = HIDHelper.getAxisMapped(1, -100, 0);
        assertEquals(0, map, 0.01);

        map = HIDHelper.getAxisMapped(-1, 50, 100);
        assertEquals(50, map, 0.01);

        map = HIDHelper.getAxisMapped(0, 50, 100);
        assertEquals(75, map, 0.01);

        map = HIDHelper.getAxisMapped(1, 50, 100);
        assertEquals(100, map, 0.01);
    }

}