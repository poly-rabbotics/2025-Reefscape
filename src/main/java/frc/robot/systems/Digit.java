package frc.robot.systems;

import org.

public class Digit() {

    private static final Digit instance = new Digit();
    private REVDigitMXPDisplay display;

    private Digit() {
        display = REVDigitMXPDisplay.getInstance();
        display.setDebug(false);
        display.clear();
    }

    public void set(String text) {
        display.displayText(text);
    }
}
