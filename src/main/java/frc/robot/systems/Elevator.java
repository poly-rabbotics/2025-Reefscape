package frc.robot.systems;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

public class Elevator{

    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double POSE_STOWED = 0.0;
    private static final double POSE_INTAKING = 0.0;
    private static final double POSE_SCORING_L1 = 0.0;
    private static final double POSE_SCORING_L2 = 0.0;
    private static final double POSE_SCORING_L3 = 0.0;
    private static final double POSE_SCORING_L4 = 0.0;

    private static final double POSE_TOLERANCE = 0.0;

    private static ElevatorPose setPose;

    private static final TalonFX elfKraken = new TalonFX(0);

    public enum ElevatorPose {
        Stowed,
        Intaking,
        ScoringL1,
        ScoringL2,
        ScoringL3,
        ScoringL4
    }

    private static final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );


    /**
     * Set the desired position of the elevator.
     */
    public static void setPose(ElevatorPose pose){
        setPose = pose;
    }

    /**
     * True if the elevator is within tolerance of its set position.
     */
    public static boolean atPose() {
        return Math.abs(getPose() - getElevatorPose(setPose)) < POSE_TOLERANCE;
    }

    /**
     * Run the elevator's devices according to its current state.
     */
    public static void run(){
        elfKraken.set(controller.calculate(getPose(), getElevatorPose(setPose)));
    }

    /**
     * Gets the current position of the elevator. Made its own method such that the source of this
     * position may be easily changed between being an internal or external sensor.
     */
    private static double getPose() {
        return elfKraken.getPosition().getValueAsDouble();
    }

    /**
     * Gets the numeric position corrosponding to the given human readable elevator position.
     */
    private static double getElevatorPose(ElevatorPose pose) {
        switch (pose) {
            case Intaking:
                return POSE_INTAKING;
            case ScoringL1:
                return POSE_SCORING_L1;
            case ScoringL2:
                return POSE_SCORING_L2;
            case ScoringL3:
                return POSE_SCORING_L3;
            case ScoringL4:
                return POSE_SCORING_L4;
            case Stowed:
            default:
                return POSE_STOWED;
        }
    }

    public static void recordState() {
        Logger.recordOutput("Elevator Set Point", setPose);
        Logger.recordOutput("Elevator Set Point (Meters)", getElevatorPose(setPose));
        Logger.recordOutput("Elevator Position", getPose());
        Logger.recordOutput("Elevator At Pose", atPose());
        Logger.recordOutput("Elevator Speed", elfKraken.get());
    }
}
