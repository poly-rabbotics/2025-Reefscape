package frc.robot.subsystems.Coral;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

public class Elevator {

    private static ElevatorPose currentPose;
    private static double currentSetpoint;

    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double SCORING_POINT_L1 = 0.0;
    private static final double SCORING_POINT_L2 = 0.0;
    private static final double SCORING_POINT_L3 = 0.0;
    private static final double SCORING_POINT_L4 = 0.0;

    private static final double MIN_SETPOINT = 0.0;
    private static final double MAX_SETPOINT = 0.0;

    private static double speed = 0.0;

    private static final TalonFX elfKraken = new TalonFX(0);

    public enum ElevatorPose {
        L1,
        L2,
        L3,
        L4,
    }

    private final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );

    public void setPose(ElevatorPose pose) {
        currentPose = pose;

        if (currentPose == ElevatorPose.L1) {
            currentSetpoint = SCORING_POINT_L1;
        } else if (currrentPose == ElevatorPose.L2) {
            currentSetpoint = SCORING_POINT_L2;
        } else if (currrentPose == ElevatorPose.L3) {
            currentSetpoint = SCORING_POINT_L3;
        } else if (currrentPose == ElevatorPose.L4) {
            currentSetpoint = SCORING_POINT_L4;
        }
    }

    public void run() {
        var pose = elfKraken.getPosition().getValueAsDouble();
        speed = controller.calculate(pose, currentSetpoint);
        elfKraken.set(speed);
    }

    /**
     * Returns the current `ElevatorPose`
     */
    public ElevatorPose getPose() {
        return currentPose;
    }

    /**
     * Returns the current position setpoint
     */
    public double getSetpoint() {
        return currentSetpoint;
    }

    /**
     * Returns the current relative encoder position of the motor
     */
    public double getPosition() {
        return elfKraken.getPosition().getValueAsDouble();
    }

    /**
     * Returns the current speed of the motor, from 0-1
     */
    public double getSpeed() {
        return elfKraken.get();
    }


    public static void recordState() {
        Logger.recordOutput("Elevator Set Point", currrentPose);
        Logger.recordOutput(
            "Elevator Position",
            elfKraken.getPosition().getValueAsDouble()
        );
        Logger.recordOutput("Elevator Speed", elfKraken.get());
    }
}
