package frc.robot.systems;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator{
    private static ElevatorPose currrentPose;

    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double L1Scaoring = 0.0;
    private static final double L2Scaoring = 0.0;
    private static final double L3Scaoring = 0.0;
    private static final double L4Scaoring = 0.0;

    private static final double MIN_SETPOINT = 0.0;
    private static final double MAX_SETPOINT = 0.0;

    private static double speed = 0.0;

    private static final TalonFX elfKraken = new TalonFX(0);
    public enum ElevatorPose {
        L1Scaoring,
        L2Scaoring,
        L3Scaoring,
        L4Scaoring  
    }

    private final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );

    public void setPose(ElevatorPose pose){
        currrentPose = pose;
    }
  
    public void run(){
        double setpoint = 0;

        if (currrentPose == ElevatorPose.L1Scaoring) {
            setpoint = L1Scaoring;
        } else if (currrentPose == ElevatorPose.L2Scaoring) {
            setpoint = L2Scaoring;
        } else if (currrentPose == ElevatorPose.L3Scaoring) {
            setpoint = L3Scaoring;
        } else if (currrentPose == ElevatorPose.L4Scaoring) {
            setpoint = L4Scaoring;
        }

        var pose = elfKraken.getPosition().getValueAsDouble();
        speed = controller.calculate(pose, setpoint);
        elfKraken.set(speed);
    }

    public static void recordState() {
        Logger.recordOutput("Elevator Set Point", currrentPose);
        Logger.recordOutput("Elevator Position", elfKraken.getPosition().getValueAsDouble());
        Logger.recordOutput("Elevator Speed", elfKraken.get());
    }
}
