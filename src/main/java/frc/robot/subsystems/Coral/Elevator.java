package frc.robot.subsystems.Coral;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.systems.Coral.IntakePivotPoint;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator{

    private final double PID_P = 0.0;
    private final double PID_I = 0.0;
    private final double PID_D = 0.0;

    private final double POSE_STOWED = 0.0;
    private final double POSE_INTAKING = 0.0;
    private final double POSE_SCORING_L1 = 0.0;
    private final double POSE_SCORING_L2 = 0.0;
    private final double POSE_SCORING_L3 = 0.0;
    private final double POSE_SCORING_L4 = 0.0;

    private final double POSE_TOLERANCE = 0.0;

    private IntakePivotPoint setPose = IntakePivotPoint.Stowed;
    private double currentSetpoint = 0;

    private final TalonFX elfKraken;

    public enum ElevatorPose {
        Stowed,
        Intaking,
        ScoringL1,
        ScoringL2,
        ScoringL3,
        ScoringL4
    }

    private final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );

    public Elevator(int motorID) {
        elfKraken = new TalonFX(motorID);
    }


    /**
     * Set the desired position of the elevator.
     */
    public void set(IntakePivotPoint pose){
        setPose = pose;

        switch (setPose) {
            case Intaking:
                currentSetpoint = POSE_INTAKING;
            case ScoringL1:
                currentSetpoint = POSE_SCORING_L1;
            case ScoringL2:
                currentSetpoint = POSE_SCORING_L2;
            case ScoringL3:
                currentSetpoint = POSE_SCORING_L3;
            case ScoringL4:
                currentSetpoint = POSE_SCORING_L4;
            case Stowed:
                currentSetpoint = POSE_STOWED;
            default:
                currentSetpoint = POSE_STOWED;
        }
    }

    /**
     * True if the elevator is within tolerance of its set position.
     */
    public boolean atPosition() {
        return Math.abs(getPosition() - currentSetpoint) < POSE_TOLERANCE;
    }

    /**
     * Run the elevator's devices according to its current state.
     */
    public void run(){
        elfKraken.set(controller.calculate(getPosition(), currentSetpoint));
    }

    //TODO: REMOVE THIS FOR MANUAL/PID MODE CONTROL
    public  void _overrideElevatorSpeed(double speed) {
        elfKraken.set(speed);
    }

    /**
     * Gets the current position of the elevator. Made its own method such that the source of this
     * position may be easily changed between being an internal or external sensor.
     */
    public double getPosition() {
        return elfKraken.getPosition().getValueAsDouble();
    }

    /**
     * Returns the setpoint position of the elevator
     */
    public double getSetpoint() {
        return currentSetpoint;
    }

    /**
     * Returns the setpoint enum of the elevator
     */
    public IntakePivotPoint getSetpointEnum() {
        return setPose;
    }

    public double getSpeed() {
        return elfKraken.get();
    }
}
