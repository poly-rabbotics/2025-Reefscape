package frc.robot.systems;

import org.team7042.REVDigitMXPDisplay;

public class Digit {

    private static enum DisplayModes {
        AUTO,
        BATTERY,
        LOL
    }

    private static final Digit instance = new Digit();
    private static org.usfirst.frc.team7042.REVDigitMXPDisplay display;
    private static DisplayModes currentDisplayMode = DisplayModes.values()[0];

    private Digit() {
        display = REVDigitMXPDisplay.getInstance();
        display.setDebug(false);
        display.clear();
    }

    public static void run() {
        display.displayScrollText("69420 NICE", 0.1);

        if(display.getAButtonPressed()) {

        }
    }

    public static double getPot() {
        return display.getPot();
    }

    private static int getModeIndex(DisplayModes mode) {
        for(int i = 0; i < DisplayModes.values().length; i++) {
            if(mode == DisplayModes.values()[i])
                return i;
        }
    }
}