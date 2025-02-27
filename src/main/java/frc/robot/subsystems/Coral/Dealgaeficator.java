package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Dealgaeficator {
    VictorSP motor;

    public Dealgaeficator(int channel) {
        motor = new VictorSP(channel);
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public double get() {
        return motor.get();
    }
}
