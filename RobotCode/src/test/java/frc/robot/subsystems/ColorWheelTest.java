package frc.robot.subsystems;

import org.junit.Test;
import frc.robot.subsystems.Lights;
import static org.junit.Assert.*;
import java.util.ArrayList;
import java.lang.Character;

public class ColorWheelTest {
    @Test
    public void colorFromRGBTest() {
        ArrayList<Character> colors = new ArrayList<>();
        for (int r = 0; r < 256; r+=10) {
            for (int g = 0; g < 256; g+=10) {
                for (int b = 0; b < 256; b+=10) {
                    final char color =  ColorWheel.getInstance().colorFromRGB(new int[]{r,g,b});
                    System.out.println(r + ", " + g + ", " + b + ", " + color);
                    colors.add(color);
                }
            }
        }
        assertNotEquals("No U", colors.indexOf('U'), -1);
        assertNotEquals("No R", colors.indexOf('R'), -1);
        assertNotEquals("No G", colors.indexOf('G'), -1);
        assertNotEquals("No B", colors.indexOf('B'), -1);
        assertNotEquals("No Y", colors.indexOf('Y'), -1);
    }
}