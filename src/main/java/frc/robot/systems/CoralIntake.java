package frc.robot.systems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Angle;

public class CoralIntake {
    private static final double INTAKE_SPEED = -0.75;
    private static final double OUTTAKE_SPEED = 0.75;

    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(2.5);

    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(110.0);
    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(210.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(0.0);

    private static final double PIVOT_PID_P = 0.0;
    private static final double PIVOT_PID_I = 0.0;
    private static final double PIVOT_PID_D = 0.0;

    private static final SparkMax rightMotor = new SparkMax(7, MotorType.kBrushless);
    private static final SparkMax leftMotor = new SparkMax(11, MotorType.kBrushless);
    private static final SparkMax pivotMotor = new SparkMax(5, MotorType.kBrushless);

    private static final PIDController pivotController
        = new PIDController(PIVOT_PID_P, PIVOT_PID_I, PIVOT_PID_D);

    private static IntakePivotPoint pivotSetPoint = IntakePivotPoint.Stowed;
    private static Angle pivotSetAngle = PIVOT_POINT_STOW;

    public enum IntakeState {
        Intaking, Outtaking, Idle
    }

    public enum IntakePivotPoint {
        ScoringL1, ScoringL2, ScoringL3, ScoringL4, Stowed, Intaking
    }

    static {
        SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
        SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig pivotEncoderConfig = new AbsoluteEncoderConfig();

        rightMotorConfig.follow(leftMotor, true);

        leftMotor.configure(
            leftMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        rightMotor.configure(
            rightMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        pivotEncoderConfig.positionConversionFactor(360.0 / 9.0);
        pivotMotorConfig.apply(pivotEncoderConfig);
        pivotMotor.configure(
            pivotMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public static void setIntakeState(IntakeState state) {
        if (state == IntakeState.Intaking) {
            leftMotor.set(INTAKE_SPEED);
        } else if (state == IntakeState.Outtaking) {
            leftMotor.set(OUTTAKE_SPEED);
        } else if (state == IntakeState.Idle){
            leftMotor.set(0);
        }
    }

    /**
     * Sets the pivot's set position.
     */
    public static void pivotTo(IntakePivotPoint setPivot) {
        pivotSetPoint = setPivot;
        pivotSetAngle = pivotAngle(setPivot);
    }

    /**
     * Returns true if the current position is within tolerance of the set position of the pivot.
     */
    public static boolean atPivotPoint() {
        return Math.abs(pivotMotor.getEncoder().getPosition() - pivotSetAngle.degrees())
            < PIVOT_TOLERANCE.degrees();
    }

    public static void run() {
        double pivotSpeed = pivotController.calculate(
            pivotMotor.getEncoder().getPosition(),
            pivotSetAngle.degrees()
        );

        pivotMotor.set(pivotSpeed);
    }

    public static void _overridePivotSpeed(double speed) {
        pivotMotor.set(speed / 10.0);
    }

    private static Angle pivotAngle(IntakePivotPoint setPivot) {
        switch (setPivot) {
            case ScoringL1:
                return PIVOT_POINT_L1;
            case ScoringL2:
                return PIVOT_POINT_L2;
            case ScoringL3:
                return PIVOT_POINT_L3;
            case ScoringL4:
                return PIVOT_POINT_L4;
            case Stowed:
            default:
                return PIVOT_POINT_STOW;
        }
    }

    public static void recordState() {
        Logger.recordOutput("Coral Intake Pivot Speed", pivotMotor.get());
        Logger.recordOutput("Coral Intake Set Angle (Degrees)", pivotSetAngle.degrees());
        Logger.recordOutput("Coral Intake Set Point", pivotSetPoint);
        Logger.recordOutput("Coral Intake Speed", leftMotor.get());
        Logger.recordOutput("Coral Intake Pivot Position", pivotMotor.getAbsoluteEncoder().getPosition());
    }
}



