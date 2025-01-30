package frc.robot.systems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Climber.*;

public class Climber {

    private static final int CLIMBER_PNEUMATIC_FORWARD_CHANNEL = 0;
    private static final int CLIMBER_PNEUMATIC_REVERSE_CHANNEL = 1;

    private static final int CLIMBER_PIVOT_LEFT_ID = 0;
    private static final int CLIMBER_PIVOT_RIGHT_ID = 1;

    private static ClimberPneumatic climberPneumatic;
    private static ClimberPivot climberPivot;

    static {
        climberPneumatic = new ClimberPneumatic(
            CLIMBER_PNEUMATIC_FORWARD_CHANNEL,
            CLIMBER_PNEUMATIC_REVERSE_CHANNEL
        );

        climberPivot = new ClimberPivot(
            CLIMBER_PIVOT_LEFT_ID,
            CLIMBER_PIVOT_RIGHT_ID,
            false,
            false
        );
    }

    public static void run(boolean pneumaticButtonPressed, double pivotSpeed) {
        // Toggle the climber pneumatic when the button is pressed
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

        climberPivot.set(pivotSpeed);

        //Run all subsystems
        climberPivot.run();
    }
}
