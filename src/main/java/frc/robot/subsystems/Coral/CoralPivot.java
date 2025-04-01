package frc.robot.subsystems.Coral;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Coral.ScoringPoint;

public class CoralPivot {
    private static final double GEAR_RATIO = 1.0 / 75.0;
    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(3.0);

    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(25.0);
    private static final Angle PIVOT_POINT_HANDOFF = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_POINT_STOW = PIVOT_POINT_INTAKE;
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(-30.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(-50.0);
    private static final Angle PIVOT_POINT_ALGAE_L2 = new Angle().setDegrees(-90.0);
    private static final Angle PIVOT_POINT_ALGAE_L3 = new Angle().setDegrees(-90.0);
    private static final Angle PIVOT_POINT_ALGAE_NET = new Angle().setDegrees(-40.0);

    private static final double PID_P = 0.01;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.000005;
    private static final double PID_FF = 0.0;
    private static final Angle MAX_G_TORQUE = new Angle().setDegrees(-90.0);
    private static final double MAX_OUTPUT_MAGNITUDE = 0.7;

    private final SparkMax motor;
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
                    .zeroOffset(188.75 / 360.0)
                    .positionConversionFactor(360.0)
                    .velocityConversionFactor(360.0)
                    .inverted(true)
                    .zeroCentered(true)
                )
                .apply(new EncoderConfig()
                    .positionConversionFactor(360.0 * GEAR_RATIO)
                    .velocityConversionFactor(360.0 * GEAR_RATIO)
                )
                .apply(new ClosedLoopConfig()
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(PID_P, PID_I, PID_D)
                    .outputRange(-MAX_OUTPUT_MAGNITUDE, MAX_OUTPUT_MAGNITUDE)
                )
                .voltageCompensation(12.0),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    /**
     * Run the pivot given its set state. Should be the last action performed in a single frame,
     * overrides not withstanding.
     */
    public void run() {
        motor
            .getClosedLoopController()
            .setReference(
                getSetAngle().degrees(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                PID_FF * Math.cos(MAX_G_TORQUE.radians() - getAngle().radians())
            );
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
        return new Angle().setDegrees(motor.getAbsoluteEncoder().getPosition());
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
            case AlgaeNet:
                return PIVOT_POINT_ALGAE_NET;
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
