package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class CoralIntakesLittleBrother {
    VictorSP motor = new VictorSP(0); //TODO: FIND THIS MOTOR'S ACTUAL ID

    public void run(double speed) {
        motor.set(speed);
    }

    public double get() {
        return motor.get();
    }
}
