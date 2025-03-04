package frc.robot.subsystems.Coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.systems.Coral.ScoringPoint;

public class Elevator {
    private static final double MIN_HEIGHT_INCHES = 0;

    private static final double GEAR_RATIO = 1.0 / (9.0 * 5.0);
    private static final double SPROCKET_DIAMETER_INCHES = 2;
    private static final double INCHES_PER_SPROCKET_ROTATION = 3 * Math.PI * SPROCKET_DIAMETER_INCHES;
    private static final double INCHES_PER_ROTATION = GEAR_RATIO * INCHES_PER_SPROCKET_ROTATION;

    private static final int ENC_PULSES_PER_CYCLE = 2048;

    private static final double MAGIC_ACCEL = 175.0;
    private static final double MAGIC_VELOCITY = 100.0;

    private static final double POSE_STOWED = 0.0;
    private static final double POSE_INTAKING = 0.0;
    private static final double POSE_HANDOFF = 0.0;
    private static final double POSE_SCORING_L1 = 0.0;
    private static final double POSE_SCORING_L2 = 0.0;
    private static final double POSE_SCORING_L3 = 23.0;
    private static final double POSE_SCORING_L4 = 58.0;
    private static final double POSE_DEALGAEFICATE_L2 = POSE_SCORING_L2;
    private static final double POSE_DEALGAEFICATE_L3 = POSE_SCORING_L3 - 2.0;

    private static final double POSE_TOLERANCE = 0.0;

    private final DigitalInput bottomSwitch;
    private final Encoder throughBore;
    private final TalonFX elfKraken;

    private ScoringPoint setPose = ScoringPoint.Stowed;

    public Elevator(int motorID, InvertedValue inverted, int bottomLimit, int encChanA, int encChanB) {
        elfKraken = new TalonFX(motorID);

        // Configure motor to give position feedback in inches extension.
        // Use `TalonFX.getPosition()`, not `TalonFX.getRotorPosition()` for elevator position.
        TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(inverted))
            .withTorqueCurrent(new TorqueCurrentConfigs()
                // These can potentially be used to reduce the maximum torque the motor exerts,
                // which may be useful to limit chain skipping as well as force on the elevator.
                .withPeakForwardTorqueCurrent(10.0)
                .withPeakReverseTorqueCurrent(-10.0)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(MAGIC_ACCEL / INCHES_PER_ROTATION)
                .withMotionMagicCruiseVelocity(MAGIC_VELOCITY / INCHES_PER_ROTATION)
            )
            .withSlot0(new Slot0Configs()
                .withKG(0.35) // Feed forward for gravity
                .withKS(0.0) // Feed forward for static friction
                .withKV(0.0) // Output per unit of desired velocity
                .withKA(0.0) // Output per unit of desired acceleration
                .withKP(0.42) // Proportional gain
                .withKI(0.0) // Intagral gain
                .withKD(0.0) // Derivative gain
            );

        elfKraken.getConfigurator().apply(config);
        elfKraken.setPosition(0.0);

        bottomSwitch = new DigitalInput(bottomLimit);

        throughBore = new Encoder(encChanA, encChanB);
        throughBore.setDistancePerPulse(INCHES_PER_SPROCKET_ROTATION / ENC_PULSES_PER_CYCLE);
        // throughBore.setIndexSource(bottomSwitch, IndexingType.kResetWhileLow);
    }

    /**
     * Run the elevator's devices according to its current state.
     */
    public void run() {
        if (atLowerLimit()) {
            setPosition(MIN_HEIGHT_INCHES);
        }

        // Motion magic control
        MotionMagicVoltage magicRequest = new MotionMagicVoltage(getSetpoint() / INCHES_PER_ROTATION);
        elfKraken.setControl(magicRequest);
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

    /**
     * Gets the current position of the elevator. Made its own method such that the source of this
     * position may be easily changed between being an internal or external sensor.
     */
    public double getPosition() {
        return throughBore.getDistance();
    }

    public double getKrakenPosition() {
        return elfKraken.getPosition().getValueAsDouble() * INCHES_PER_ROTATION;
    }

    /**
     * Sets the elevator's position, and resets the through bore if set to zero.
     */
    public void setPosition(double pose) {
        elfKraken.setPosition(pose / INCHES_PER_ROTATION);

        if (pose == 0.0) {
            throughBore.reset();
        }
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

    /**
     * Gets the set speed of the elevator's motor.
     */
    public double getSpeed() {
        return elfKraken.get();
    }

    /**
     * Gets the velocity of the elevator.
     */
    public double getVelocity() {
        return elfKraken.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the velocity of the elevator.
     */
    public double getAcceleration() {
        return elfKraken.getAcceleration().getValueAsDouble();
    }

    /**
     * True if the lower limit switch is flipped.
     */
    public boolean atLowerLimit() {
        return !bottomSwitch.get();
    }
    
    /**
     * Manually overrides the speed of the elevator's motor, do not use unless absolutely required.
     */
    public void _setSpeed(double speed) {
        elfKraken.set(speed);
    }

    /**
     * Get an extension, in inches, based on the given `ScoringPoint`.
     */
    private static double extensionFromScoringPose(ScoringPoint pose) {
        switch (pose) {
            case Intaking:
                return POSE_INTAKING;
            case Handoff:
                return POSE_HANDOFF;
            case ScoringL1:
                return POSE_SCORING_L1;
            case ScoringL2:
                return POSE_SCORING_L2;
            case ScoringL3:
                return POSE_SCORING_L3;
            case ScoringL4:
                return POSE_SCORING_L4;
            case DealgaeficateL2:
                return POSE_DEALGAEFICATE_L2;
            case DealgaeficateL3:
                return POSE_DEALGAEFICATE_L3;
            case Stowed:
            default:
                return POSE_STOWED;
        }
    }
}
