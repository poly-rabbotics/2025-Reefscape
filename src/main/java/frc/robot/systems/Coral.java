package frc.robot.systems;

import frc.robot.subsystems.IntakeCoral;

public class Coral {

    private static final Coral instance = new Coral();
    public static IntakeCoral intakeCoral;

    private Coral() {
        intakeCoral = new IntakeCoral(0, 0);
    }

    public static void run(double intakeSpeed) {
        intakeCoral.set(intakeSpeed);

        intakeCoral.run();
    }
}
