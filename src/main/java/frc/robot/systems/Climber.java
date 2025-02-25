package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Climber.*;

public class Climber {

    private static final int CLIMBER_PIVOT_ID = 2;

    private static ClimberPivot climberPivot;

    static {

        climberPivot = new ClimberPivot(
            CLIMBER_PIVOT_ID,
            false,
            false
        );
    }

    public static void run(boolean runPID, double pivotSpeed) {

        climberPivot.set(pivotSpeed);

        //Run all subsystems
        climberPivot.run(runPID);
    }

    public static void recordState() {
        Logger.recordOutput("Climber Pivot Position (Degrees)", climberPivot.getPosition().degrees());
        Logger.recordOutput("Climber Motor Speed", climberPivot.get());;
    }
}
