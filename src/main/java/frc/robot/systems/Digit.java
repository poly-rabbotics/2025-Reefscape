package frc.robot.systems;

import org.usfirst.frc.team7042.REVDigitMXPDisplay;

public class Digit {

    private static enum DisplayMode {
        AUTO,
        BATTERY,
        LOL
    }

    private static final Digit instance = new Digit();
    private static org.usfirst.frc.team7042.REVDigitMXPDisplay display;
    private static DisplayMode currentDisplayMode = DisplayMode.values()[0];

    private Digit() {
        display = REVDigitMXPDisplay.getInstance();
        display.setDebug(false);
        display.clear();
    }

    public static void run() {
        if(display.getAButtonPressed()) {
            currentDisplayMode = getNextMode(currentDisplayMode, true);
        } else if(display.getBButtonPressed()) {
            currentDisplayMode = getNextMode(currentDisplayMode, false);
        }

        switch (currentDisplayMode) {
            case AUTO:
                autoModeRun();
            case BATTERY:
                batteryModeRun();
            case LOL: 
                lolModeRun();


        }

    }

    public static double getPot() {
        return display.getPot();
    }

    private static DisplayMode getNextMode(DisplayMode mode, boolean increase) {
        int index = 0;

        for(int i = 0; i < DisplayMode.values().length; i++) {
            if(mode == DisplayMode.values()[i])
                index = i;
                break;
        }

        if(increase) {
            if(index >= DisplayMode.values().length - 1) {
                index = 0;
            } else {
                index++;
            }
        } else {
            if(index <= 0) {
                index = DisplayMode.values().length - 1;
            } else {
                index--;
            }
        }

        return DisplayMode.values()[index];
    }

    private static void autoModeRun() {
        display.displayText("1");
    }

    private static void batteryModeRun() {
        display.displayText("2");
    }

    private static void lolModeRun() {
        display.displayText("3");
    }
}