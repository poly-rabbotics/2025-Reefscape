package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class CoralIntakesLittleBrother {
    VictorSP motor;

    public CoralIntakesLittleBrother(int channel) {
        motor = new VictorSP(channel);
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public double get() {
        return motor.get();
    }
}
