package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClimberPneumatic {

    private DoubleSolenoid climberSolenoid;

    public ClimberPneumatic(int forwardChannel, int reverseChannel) {
        climberSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            forwardChannel,
            reverseChannel
        );

        climberSolenoid.set(Value.kForward);
    }

    public void set(Value val) {
        climberSolenoid.set(val);
    }

    public DoubleSolenoid.Value get() {
        return climberSolenoid.get();
    }
}
