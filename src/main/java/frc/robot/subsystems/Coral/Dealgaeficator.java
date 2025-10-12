package frc.robot.subsystems.Coral;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import frc.robot.systems.Pose;

public class Dealgaeficator {
    VictorSP motor;
    Joystick buttonPanel;

    public Dealgaeficator(int channel) {
        motor = new VictorSP(channel);
        buttonPanel = new Joystick(3);
    }

    public void run()
    {
        if (buttonPanel.getRawButton(9)||Pose.pose=="AlXFR"||Pose.pose=="AlXFRPrep") // orignally 7
        {
            set(-1);
        }
        else if(buttonPanel.getRawButton(8)) // originally 6
        {
            set(1);
        }
        else {
            set(0);
        }
    }

    public void set(double speed) {
        motor.set(speed);
    }

    public double get() {
        return motor.get();
    }
}
