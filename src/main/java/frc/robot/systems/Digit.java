package frc.robot.systems;

import org.usfirst.frc.team3487.REVDigitMXPDisplay;

public class Digit {

    private static final Digit instance = new Digit();
    private static REVDigitMXPDisplay display;

    private Digit() {
        display = REVDigitMXPDisplay.getInstance();
        display.setDebug(false);
        display.clear();
    }

    public static void run() {
        display.displayScrollText("69420 NICE", 0.1);
    }

    public static double getPot() {
        return display.getPot();
    }

}