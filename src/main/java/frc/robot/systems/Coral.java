package frc.robot.systems;

import frc.robot.subsystems.CoralIntake;

public class Coral {

    private static final Coral instance = new Coral();
    public static CoralIntake coralIntake;

    private Coral() {
        coralIntake = new CoralIntake(0, 0);
    }

    public static void run(double intakeSpeed) {
        coralIntake.run(intakeSpeed);
    }

}
