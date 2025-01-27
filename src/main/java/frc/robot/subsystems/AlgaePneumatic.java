package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class AlgaePneumatic {

    private DoubleSolenoid algaeSolenoid;

    public AlgaePneumatic(int forwardChannel, int reverseChannel) {
        algaeSolenoid = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            forwardChannel,
            reverseChannel
        );

        algaeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void set(Value val) {
        algaeSolenoid.set(val);
    }

    public DoubleSolenoid.Value get() {
        return algaeSolenoid.get();
    }
}
