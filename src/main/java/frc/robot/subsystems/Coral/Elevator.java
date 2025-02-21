package frc.robot.subsystems.Coral;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.systems.Coral.ScoringPoint;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator{

    private final double MIN_HEIGHT = 0;
    private final double MAX_HEIGHT = 51; // In inches

    private final double TICKS_PER_INCH = 1; // Constant representing the number of motor encoder ticks per inch of vertical translation on the elevator.

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

    private ScoringPoint setPose = ScoringPoint.Stowed;
    private double currentSetpoint = 0;

    private final DigitalInput bottomSwitch = new DigitalInput(1);
    private final DigitalInput topSwitch = new DigitalInput(1);

    private final TalonFX elfKraken;

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
    public void set(ScoringPoint pose){
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
        if(bottomSwitch.get()) {
            elfKraken.setPosition(MIN_HEIGHT * TICKS_PER_INCH);
        } else if(topSwitch.get()) {
            elfKraken.setPosition(MAX_HEIGHT * TICKS_PER_INCH)
        }


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
    public ScoringPoint getSetpointEnum() {
        return setPose;
    }

    public double getSpeed() {
        return elfKraken.get();
    }
}
