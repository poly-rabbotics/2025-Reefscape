package frc.robot.subsystems.Algae;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class AlgaePneumatic {

    private DoubleSolenoid algaeSolenoidOne;
    private DoubleSolenoid algaeSolenoidTwo;

    public AlgaePneumatic(
        int forwardChannelOne,
        int reverseChannelOne,
        int forwardChannelTwo,
        int reverseChannelTwo
    ) {
        algaeSolenoidOne = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            forwardChannelOne,
            reverseChannelOne
        );

        algaeSolenoidTwo = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM,
            forwardChannelTwo,
            reverseChannelTwo
        );

        algaeSolenoidOne.set(DoubleSolenoid.Value.kForward);
        algaeSolenoidTwo.set(DoubleSolenoid.Value.kForward);
    }

    public void set(Value val) {
        algaeSolenoidOne.set(val);
        algaeSolenoidTwo.set(val);
    }

    public DoubleSolenoid.Value get() {
        return algaeSolenoidOne.get();
    }
}
