package frc.robot.systems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePneumatic;

public class Algae {

    private static final int ALGAE_PNEUMATIC_ONE_FORWARD_CHANNEL = 0;
    private static final int ALGAE_PNEUMATIC_ONE_REVERSE_CHANNEL = 1;
    private static final int ALGAE_PNEUMATIC_TWO_FORWARD_CHANNEL = 0;
    private static final int ALGAE_PNEUMATIC_TWO_REVERSE_CHANNEL = 1;

    private static final int ALGAE_INTAKE_MOTOR_ID = 0;

    private static AlgaePneumatic algaePneumatic;
    private static AlgaeIntake algaeIntake;

    static {
        algaePneumatic = new AlgaePneumatic(
            ALGAE_PNEUMATIC_ONE_FORWARD_CHANNEL,
            ALGAE_PNEUMATIC_ONE_REVERSE_CHANNEL,
            ALGAE_PNEUMATIC_TWO_FORWARD_CHANNEL,
            ALGAE_PNEUMATIC_TWO_REVERSE_CHANNEL
        );

        algaeIntake = new AlgaeIntake(ALGAE_INTAKE_MOTOR_ID);
    }

    public static void run(boolean pneumaticButtonPressed, double intakeSpeed) {
        // Set intake speed
        algaeIntake.set(intakeSpeed);

        // Toggle the algae pneumatic when the button is pressed
        if (pneumaticButtonPressed) {
            switch (algaePneumatic.get()) {
                case kForward:
                    algaePneumatic.set(DoubleSolenoid.Value.kReverse);
                    break;
                case kReverse:
                    algaePneumatic.set(DoubleSolenoid.Value.kForward);
                    break;
                default:
                    break;
            }
        }

        algaeIntake.run();
    }
}
