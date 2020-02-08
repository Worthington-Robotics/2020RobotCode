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

}