package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.systems.Coral.ScoringPoint;

public class Elevator {
    private static final double MIN_HEIGHT_INCHES = 0;
    private static final double MAX_HEIGHT_INCHES = 51;

    private static final double GEAR_RATIO = 1.0 / (9.0 * 5.0);
    private static final double SPROCKET_DIAMETER_INCHES = 2;
    private static final double INCHES_PER_ROTATION = GEAR_RATIO * Math.PI * SPROCKET_DIAMETER_INCHES;

    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double POSE_STOWED = 0.0;
    private static final double POSE_INTAKING = 0.0;
    private static final double POSE_SCORING_L1 = 0.0;
    private static final double POSE_SCORING_L2 = 0.0;
    private static final double POSE_SCORING_L3 = 5.5;
    private static final double POSE_SCORING_L4 = 48.0;

    private static final double POSE_TOLERANCE = 0.0;

    private final DigitalInput bottomSwitch = new DigitalInput(0);
    private final DigitalInput topSwitch = new DigitalInput(1);
    private final TalonFX elfKraken;

    private final PIDController controller = new PIDController(
        PID_P,
        PID_I,
        PID_D
    );

    private ScoringPoint setPose = ScoringPoint.Stowed;

    public Elevator(int motorID, InvertedValue inverted) {
        elfKraken = new TalonFX(motorID);

        // Configure motor to give position feedback in inches extension.
        // Use `TalonFX.getPosition()`, not `TalonFX.getRotorPosition()` for elevator position.
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(inverted))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                // These can potentially be used to reduce the maximum torque the motor exerts,
                // which may be useful to limit chain skipping as well as force on the elevator.
                .withPeakForwardTorqueCurrent(30.0)
                .withPeakReverseTorqueCurrent(-30.0)
            );
        elfKraken.getConfigurator().apply(config);
    }

    /**
     * Run the elevator's devices according to its current state.
     */
    public void run(){
        if (bottomSwitch.get()) {
            setPosition(MIN_HEIGHT_INCHES);
        } else if (topSwitch.get()) {
            setPosition(MAX_HEIGHT_INCHES);
        }

        elfKraken.set(controller.calculate(getPosition(), getSetpoint()));
    }

    /**
     * Set the desired position of the elevator.
     */
    public void set(ScoringPoint pose){
        setPose = pose;
    }


    /**
     * True if the elevator is within tolerance of its set position.
     */
    public boolean atPosition() {
        return Math.abs(getPosition() - extensionFromScoringPose(setPose)) < POSE_TOLERANCE;
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
        return elfKraken.getPosition().getValueAsDouble() * INCHES_PER_ROTATION;
    }

    public void setPosition(double pose) {
        elfKraken.setPosition(pose / INCHES_PER_ROTATION);
    }

    /**
     * Returns the setpoint position of the elevator
     */
    public double getSetpoint() {
        return extensionFromScoringPose(setPose);
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

    public double getVelocity() {
        return elfKraken.getVelocity().getValueAsDouble();
    }
    /**
     * Get an extension, in inches, based on the given `ScoringPoint`.
     */
    public static double extensionFromScoringPose(ScoringPoint pose) {
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
}
