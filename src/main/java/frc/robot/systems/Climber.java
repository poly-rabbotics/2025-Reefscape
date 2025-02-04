package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Climber.*;

public class Climber {
    private static final int CLIMBER_PNEUMATIC_FORWARD_CHANNEL = 8;
    private static final int CLIMBER_PNEUMATIC_REVERSE_CHANNEL = 9;

    private static final int CLIMBER_PIVOT_LEFT_ID = 2;
    private static final int CLIMBER_PIVOT_RIGHT_ID = 3;

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

    public static void recordState() {
        Logger.recordOutput("Climber Piston State", climberPneumatic.get());
        Logger.recordOutput("Climber Pivot Position (Degrees)", climberPivot.getPosition().degrees());
    }
}
