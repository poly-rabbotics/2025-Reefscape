package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.systems.Coral;
import frc.robot.systems.Coral.ScoringPoint;

public class CoralIntake {
    private static final double MOTOR_GEAR_RATIO = 1.0 / 15.0;
    private static final double SPROCKET_RATIO = 15.0 / 18.0;
    private static final double WHEEL_CIRCUMFRANCE = 2.25 * Math.PI;

    /*
     * Positions are in inches, which zero being the entrance of the intake. Negative position
     * indicate moving further into the intake, while positive positions exist past the font og the
     * intake.
     *
     * Positions track the outside edge of the coral. The front of the intake is taken to be the
     * center of the first set of wheels. Sensors are to be considered infinitely small points,
     * meaning that a trip and untrip indicate the coral being at the same position. This class
     * considers the detection width negligable.
     *
     * The spacing between prox sensor mount points is 1.05", and the first appears about 1.35" away
     * from the center point of the front rollers along the coral's axis of movement within the
     * intake. Any prox sensor position should be findable by multiplying 1.05" by the number of
     * spaces to the front of where the sensor is mounted and adding that to 1.35".
     */

    private static final double FRONT_PROX_POSE = -1.35;
    private static final double BACK_PROX_POSE = -1.35 + -1.05 * 1;
    private static final double CORAL_LENGTH = 11.0 + 7.0 / 8.0;
    private static final double CORAL_STORAGE_POSE = 6.0;

    private static final double CORAL_POSE_TOLERANCE = 0.25;
   
    private static final double INTAKE_SPEED = -1.0;
    private static final double OUTTAKE_SPEED = 1.0;

    private static final double PID_P = 0.035;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private SparkMax bottomMotor;
    private SparkMax topMotor;
    private DigitalInput frontProx;
    private DigitalInput backProx;

    private boolean frontProxLastState;
    private boolean backProxLastState;

    private IntakeState currentState = IntakeState.Idle;
    private PIDController controller = new PIDController(PID_P, PID_I, PID_D);

    public enum IntakeState {
        Storing, Intaking, Outtaking, Idle
    }

    public CoralIntake(
        int bottomMotorID,
        int topMotorID,
        int frontProxChannel,
        int backProxChannel
    ) {
        bottomMotor = new SparkMax(bottomMotorID, MotorType.kBrushless);
        topMotor = new SparkMax(topMotorID, MotorType.kBrushless);
        frontProx = new DigitalInput(frontProxChannel);
        backProx = new DigitalInput(backProxChannel);

        topMotor.configure(
            new SparkMaxConfig()
                .apply(new EncoderConfig()
                    .positionConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                    .velocityConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                )
                .inverted(true),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        bottomMotor.configure(
            new SparkMaxConfig()
                .follow(topMotor, true)
                .apply(new EncoderConfig()
                    .positionConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                    .velocityConversionFactor(MOTOR_GEAR_RATIO * SPROCKET_RATIO * WHEEL_CIRCUMFRANCE)
                ),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        frontProxLastState = frontProx.get();
        backProxLastState = backProx.get();
    }

    /**
     * Runs the intake and performs position control, should be called periodically.
     */
    public void run() {
        if (proxTripped(frontProx.get(), frontProxLastState) && !proxOn(backProx.get())) {
            // back edge of coral is in the intake, front edge a coral's width beyond the front
            // sensor.

            topMotor.getEncoder().setPosition(FRONT_PROX_POSE + CORAL_LENGTH);

            if (currentState == IntakeState.Intaking) {
                currentState = IntakeState.Storing;
            }
        }

        if (proxTripped(backProx.get(), backProxLastState) && proxOn(frontProx.get())) {
            // Coral just met the back sensor and is still on the front sensor, putting the coral's
            // front a coral's width away from the back sensors position.
            
            topMotor.getEncoder().setPosition(BACK_PROX_POSE + CORAL_LENGTH);
        }

        if (proxTripped(frontProx.get(), frontProxLastState) && proxOn(backProx.get())) {
            // Coral just left the front sensor going further into the intake, ergo its front is at
            // the front sensor's position.
            
            topMotor.getEncoder().setPosition(FRONT_PROX_POSE);
        }

        if (proxTripped(backProx.get(), backProxLastState) && !proxOn(frontProx.get())) {
            // Coral just left the back prox sensor and the front sensor is not on, meaning  that it
            // has retracted into the intake past the back sensor. This is probably not physically
            // pheasable, but in the event that the back stop breaks this position will help PID
            // keep the piece at the right position.
            
            // topMotor.getEncoder().setPosition(BACK_PROX_POSE);
        }

        double speed;

        switch (currentState) {
            case Intaking:
                speed = INTAKE_SPEED;
                break;

            case Outtaking:
                speed = OUTTAKE_SPEED;
                break;

            case Storing:
                double position = topMotor.getEncoder().getPosition();

                if (Coral.getScoringPoint() == ScoringPoint.ScoringL4) {
                    speed = controller.calculate(position, CORAL_STORAGE_POSE - 6.0);
                } else {
                    speed = controller.calculate(position, CORAL_STORAGE_POSE);
                }
                break;
                
            default:
            case Idle:
                speed = 0.0;
                break;
        }

        topMotor.set(speed);

        frontProxLastState = frontProx.get();
        backProxLastState = backProx.get();
    }

    /**
     * Sets the current state to `Intaking`.
     *
     * NOTE: DO NOT CALL STATE SETTING METHODS PERIODICALLY. This class changes its own state
     * internally and to states which are not settable publically, setting a state periodically
     * impares this behavior.
     */
    public void setIntaking() {
        currentState = IntakeState.Intaking;
    }

    /**
     * Sets the current state to `Outtaking`.
     *
     * NOTE: DO NOT CALL STATE SETTING METHODS PERIODICALLY. This class changes its own state
     * internally and to states which are not settable publically, setting a state periodically
     * impares this behavior.
     */
    public void setOuttaking() {
        currentState = IntakeState.Outtaking;
    }

    /**
     * Sets the current state to `Idle`.
     *
     * NOTE: DO NOT CALL STATE SETTING METHODS PERIODICALLY. This class changes its own state
     * internally and to states which are not settable publically, setting a state periodically
     * impares this behavior.
     */
    public void setIdle() {
        currentState = IntakeState.Idle;
    }

    /**
     * Get the set speed of the motors.
     */
    public double get() {
        return topMotor.get();
    }

    /**
     * Gets the output current given to the motors by their controllers.
     */
    public double getCurrent() {
        return topMotor.getOutputCurrent();
    }

    /**
     * Gets the motor tempurature.
     */
    public double getMotorTemp() {
        return topMotor.getMotorTemperature();
    }

    /**
     * Get the intake's current state.
     */
    public IntakeState getState() {
        return currentState;
    }

    /**
     * Returns the position of the coral piece if it is known, otherwise returning
     * `Double.POSITIVE_INFINITY` if the position is outside the intake/unknowable.
     */
    public double getCoralPosition() {
        if (proxOn(frontProx.get()) || proxOn(backProx.get())) {
            return topMotor.getEncoder().getPosition();
        }

        return Double.POSITIVE_INFINITY;
    }

    public void setCoralPosition(double coralExtension) {
        topMotor.getEncoder().setPosition(coralExtension);
        currentState = IntakeState.Storing;
    }

    /**
     * True if the coral is believed to be at its store point.
     */
    public boolean isCoralStored() {
        return Math.abs(topMotor.getEncoder().getPosition() - CORAL_STORAGE_POSE)
            < CORAL_POSE_TOLERANCE;
    }

    public boolean frontProxOn() {
        return proxOn(frontProx.get());
    }

    public boolean backProxOn() {
        return proxOn(backProx.get());
    }

    public void _setSpeed(double speed) {
        topMotor.set(speed);
    }

    /**
     * True if the sensor has changed state, whether falling or rising edge. This will therefore
     * return true if a sensor *begins* to see a piece, *or* if it *stops* seeing a piece.
     */
    private static boolean proxTripped(boolean state, boolean lastState) {
        // Floating high, falling edge
        return state != lastState;
    }

    /**
     * Whether or not the sensor is on.
     */
    private static boolean proxOn(boolean state) {
        // Floating high
        return !state;
    }
}
