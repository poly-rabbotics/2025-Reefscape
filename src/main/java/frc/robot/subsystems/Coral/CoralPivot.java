package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.subsystems.Angle;
import frc.robot.systems.Coral.ScoringPoint;

public class CoralPivot {
    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(2.5);
    private static final Angle PIVOT_OFFSET = new Angle().setDegrees(262.0);

    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(90.0);
    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(42.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(-20.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(-25.0);

    private static final double PID_P = 0.0;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final PIDController controller = new PIDController(PID_P, PID_I, PID_D);

    private final SparkMax motor;
    private ScoringPoint scoringPoint = ScoringPoint.Stowed;

    public CoralPivot(int motorId) {
        motor = new SparkMax(5, MotorType.kBrushless);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();

        encoderConfig.positionConversionFactor(360.0);
        motorConfig.apply(encoderConfig);
        motor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public void run() {
        double speed = controller.calculate(
            motor.getEncoder().getPosition(),
            pivotAngle(scoringPoint).degrees()
        );

        motor.set(speed);
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

    public Angle getAngle() {
        return new Angle()
            .setDegrees(-motor.getAbsoluteEncoder().getPosition())
            .sub(PIVOT_OFFSET);
    }

    public Angle getSetAngle() {
        return pivotAngle(scoringPoint);
    }

    public ScoringPoint getSetPoint() {
        return scoringPoint;
    }

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
            case Intaking:
                return PIVOT_POINT_INTAKE;
            case Stowed:
            default:
                return PIVOT_POINT_STOW;
        }
    }

}
