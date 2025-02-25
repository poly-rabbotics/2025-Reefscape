package frc.robot.systems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.Climber.*;

public class Climber {
    private static final int CLIMBER_PNEUMATIC_FORWARD_CHANNEL = 0;
    private static final int CLIMBER_PNEUMATIC_REVERSE_CHANNEL = 1;

    private static final int LOCK_FORWARD_CHANNEL = 2;
    private static final int LOCK_REVERSE_CHANNEL = 3;

    private static final int CLIMBER_PIVOT_ID = 2;

    private static ClimberPneumatic climberPneumatic;
    private static ClimberPivot climberPivot;
    private static DoubleSolenoid lockSolenoid;

    static {
        climberPneumatic = new ClimberPneumatic(
            CLIMBER_PNEUMATIC_FORWARD_CHANNEL,
            CLIMBER_PNEUMATIC_REVERSE_CHANNEL
        );

        climberPivot = new ClimberPivot(
            CLIMBER_PIVOT_ID,
            false,
            false
        );

        lockSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, LOCK_FORWARD_CHANNEL, LOCK_REVERSE_CHANNEL);
    }

    public static void run(boolean pneumaticButtonPressed, boolean runPID, double pivotSpeed, boolean lock) {
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

        if(lock) {
            lockSolenoid.set(Value.kForward);
        } else {
            lockSolenoid.set(Value.kReverse);
        }

        climberPivot.set(pivotSpeed);

        //Run all subsystems
        climberPivot.run(runPID);
    }

    public static void recordState() {
        Logger.recordOutput("Climber Piston State", climberPneumatic.get());
        Logger.recordOutput("Climber Pivot Position (Degrees)", climberPivot.getPosition().degrees());
        Logger.recordOutput("Climber Motor Speed", climberPivot.get());;
    }
}
