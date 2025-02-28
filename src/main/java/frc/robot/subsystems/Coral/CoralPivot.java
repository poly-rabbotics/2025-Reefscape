package frc.robot.subsystems.Coral;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Coral.ScoringPoint;

public class CoralPivot {
    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(5.0);
    private static final Angle PIVOT_OFFSET = new Angle().setDegrees(262.0 - 165.5);

    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(32.0);
    private static final Angle PIVOT_POINT_HANDOFF = new Angle().setDegrees(45.0);
    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(32.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(-7.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(-40.0);
    private static final Angle PIVOT_POINT_ALGAE_L2 = new Angle().setDegrees(-70.0);
    private static final Angle PIVOT_POINT_ALGAE_L3 = new Angle().setDegrees(-70.0);

    private static final double PID_P = 0.01;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final PIDController controller = new PIDController(PID_P, PID_I, PID_D);

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
                .idleMode(IdleMode.kBrake)
                .apply(new AbsoluteEncoderConfig()),
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        controller.enableContinuousInput(0.0, 360.0);
        controller.setTolerance(PIVOT_OFFSET.degrees());
    }

    /**
     * Run the pivot given its set state. Should be the last action performed in a single frame,
     * overrides not withstanding.
     */
    public void run() {
        double setAngle = pivotAngle(scoringPoint).degrees() % 360.0;
        double speed = controller.calculate(
            getAngle().degrees() % 360.0,
            setAngle % 360.0
        );

        if (speed > 0.5) {
            speed = 0.5;
        } else if (speed < -0.5) {
            speed = -0.5;
        }

        motor.set(speed);
        Logger.recordOutput("Coral Pivot Set Neo Speed", speed);
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
        return new Angle()
            .setDegrees(
                (-motor.getAbsoluteEncoder().getPosition() * 360.0 - PIVOT_OFFSET.degrees() + 360.0) % 360.0
            );
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
        motor.set(speed);
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
