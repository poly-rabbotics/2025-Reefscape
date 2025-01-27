package frc.robot.systems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.ClimberPneumatic;

public class Climber {

    private static final int CLIMBER_PNEUMATIC_FORWARD_CHANNEL = 0;
    private static final int CLIMBER_PNEUMATIC_REVERSE_CHANNEL = 1;

    private static ClimberPneumatic climberPneumatic;

    private static final Climber instance = new Climber();

    private Climber() {
        climberPneumatic = new ClimberPneumatic(
            CLIMBER_PNEUMATIC_FORWARD_CHANNEL,
            CLIMBER_PNEUMATIC_REVERSE_CHANNEL
        );
    }

    public static void run(boolean pneumaticButtonPressed) {
        // Toggle the algae pneumatic when the button is pressed
        if (pneumaticButtonPressed) {
            switch (climberPneumatic.get()) {
                case kForward:
                    climberPneumatic.set(DoubleSolenoid.Value.kReverse);
                    break;
                case kReverse:
                    climberPneumatic.set(DoubleSolenoid.Value.kForward);
                    break;
                default:
                    break;
            }
        }
    }
}
