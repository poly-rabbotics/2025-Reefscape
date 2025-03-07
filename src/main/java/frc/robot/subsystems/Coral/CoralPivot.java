package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Coral.ScoringPoint;

public class CoralPivot {
    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(5.0);

    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(32.0);
    private static final Angle PIVOT_POINT_HANDOFF = new Angle().setDegrees(45.0);
    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(32.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(-7.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(-40.0);
    private static final Angle PIVOT_POINT_ALGAE_L2 = new Angle().setDegrees(-70.0);
    private static final Angle PIVOT_POINT_ALGAE_L3 = new Angle().setDegrees(-70.0);

    private final SparkMax motor;
    private final SparkClosedLoopController controller;
    private ScoringPoint scoringPoint = ScoringPoint.Stowed;

    /**
     * Instantiate a coral pivot on the given Spark Max ID.
     */
    public CoralPivot(int motorId) {
        motor = new SparkMax(motorId, MotorType.kBrushless);

        motor.configure(
            new SparkMaxConfig()
                .inverted(true)
                .apply(new AbsoluteEncoderConfig()
                    .zeroOffset(0.0)
                    .positionConversionFactor(360.0)
                    .velocityConversionFactor(360.0)
                    .inverted(true)
                    .zeroCentered(true)
                )
                .apply(new EncoderConfig()
                    .positionConversionFactor(360.0 / 75.0)
                    .velocityConversionFactor(360.0 / 75.0)
                )
                .apply(new ClosedLoopConfig()
                    .apply(new MAXMotionConfig()
                        // .allowedClosedLoopError(PIVOT_TOLERANCE.degrees())
                        .maxAcceleration(150.0 * 60.0)
                        .maxVelocity(150.0 * 60.0)
                        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
                    )
                    .pid(0.016, 0.0, 0.0001)
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(-180.0, 180.0)
                ),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        controller = motor.getClosedLoopController();
        motor.getEncoder().setPosition(motor.getAbsoluteEncoder().getPosition());
    }

    /**
     * Run the pivot given its set state. Should be the last action performed in a single frame,
     * overrides not withstanding.
     */
    public void run() {
        double setAngle = pivotAngle(scoringPoint).degrees();
        controller.setReference(setAngle, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Sets the pivot's set position.
     */
    public void pivotTo(ScoringPoint setPivot) {
        scoringPoint = setPivot;
    }

    /**
     * Returns true if the current position is within tolerance of the set position of the pivot.
     */
    public boolean atPivotPoint() {
        return Math.abs(getAngle().degrees() - getSetAngle().degrees()) < PIVOT_TOLERANCE.degrees();
    }

    /**
     * Get motor power.
     */
    public double get() {
        return motor.get();
    }

    /**
     * Gets the curret angle of the pivot, as measured by a through-bore encoder.
     */
    public Angle getAngle() {
        return new Angle().setDegrees(motor.getEncoder().getPosition());
    }

    /**
     * Gets the current set angle.
     */
    public Angle getSetAngle() {
        return pivotAngle(scoringPoint);
    }

    /**
     * Gets the current set point.
     */
    public ScoringPoint getSetPoint() {
        return scoringPoint;
    }

    /**
     * Gets the motor velocity.
     */
    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    /**
     * Overrides the speed, do not use unless absolutely needed.
     */
    public void _setSpeed(double speed) {
        // motor.set(speed);
    }
    
    /**
     * Gives the pivot `Angle` associated with the given `ScoringPoint`.
     */
    private static Angle pivotAngle(ScoringPoint setPivot) {
        switch (setPivot) {
            case ScoringL1:
                return PIVOT_POINT_L1;
            case ScoringL2:
                return PIVOT_POINT_L2;
            case ScoringL3:
                return PIVOT_POINT_L3;
            case ScoringL4:
                return PIVOT_POINT_L4;
            case DealgaeficateL2:
                return PIVOT_POINT_ALGAE_L2;
            case DealgaeficateL3:
                return PIVOT_POINT_ALGAE_L3;
            case Intaking:
                return PIVOT_POINT_INTAKE;
            case Handoff:
                return PIVOT_POINT_HANDOFF;
            case Stowed:
            default:
                return PIVOT_POINT_STOW;
        }
    }

}
