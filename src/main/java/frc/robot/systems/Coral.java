package frc.robot.systems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Coral.*;
import frc.robot.subsystems.Angle;

public class Coral {
    private static final double INTAKE_SPEED = -0.75;
    private static final double OUTTAKE_SPEED = 0.75;

    private static final Angle PIVOT_TOLERANCE = new Angle().setDegrees(2.5);

    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(110.0);
    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(210.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(0.0);

    private static final SparkMax pivotMotor = new SparkMax(5, MotorType.kBrushless);
    private static final CoralIntake coralIntake = new CoralIntake(7, 11);
    private static final Elevator elevator = new Elevator(14, InvertedValue.Clockwise_Positive);

    private static final double PIVOT_PID_P = 0.0;
    private static final double PIVOT_PID_I = 0.0;
    private static final double PIVOT_PID_D = 0.0;

    private static final PIDController pivotController
        = new PIDController(PIVOT_PID_P, PIVOT_PID_I, PIVOT_PID_D);

    private static ScoringPoint currentScoringPoint = ScoringPoint.Stowed;

    public enum IntakeState {
        Intaking, Outtaking, Idle
    }

    public enum ScoringPoint {
        ScoringL1, ScoringL2, ScoringL3, ScoringL4, Stowed, Intaking
    }

    static {
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig pivotEncoderConfig = new AbsoluteEncoderConfig();

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
            coralIntake.set(INTAKE_SPEED);
        } else if (state == IntakeState.Outtaking) {
            coralIntake.set(OUTTAKE_SPEED);
        } else if (state == IntakeState.Idle){
            coralIntake.set(0);
        }
    }

    /**
     * Sets the pivot's set position.
     */
    public static void pivotTo(ScoringPoint setPivot) {
        currentScoringPoint = setPivot;
    }

    /**
     * Returns true if the current position is within tolerance of the set position of the pivot.
     */
    public static boolean atPivotPoint() {
        return Math.abs(pivotMotor.getEncoder().getPosition() - pivotAngle(currentScoringPoint).degrees())
            < PIVOT_TOLERANCE.degrees();
    }

    /**
     * To be run from teleop
     * @param intaking
     * @param outtaking
     * @param stowedPose
     * @param intakePose
     * @param l1Pose
     * @param l2Pose
     * @param l3Pose
     * @param l4Pose
     */
    public static void run(boolean intaking, boolean outtaking, boolean stowedPose, boolean intakePose, boolean l1Pose, boolean l2Pose, boolean l3Pose, boolean l4Pose) {
        // Coral intake
        if (intaking) {
            setIntakeState(IntakeState.Intaking);
        } else if (outtaking) {
            setIntakeState(IntakeState.Outtaking);
        } else {
            setIntakeState(IntakeState.Idle);
        }

        if(stowedPose) {
            currentScoringPoint = ScoringPoint.Stowed;
        } else if(intakePose) {
            currentScoringPoint = ScoringPoint.Intaking;
        } else if(l1Pose) {
            currentScoringPoint = ScoringPoint.ScoringL1;
        } else if(l2Pose) {
            currentScoringPoint = ScoringPoint.ScoringL2;
        } else if(l3Pose) {
            currentScoringPoint = ScoringPoint.ScoringL3;
        } else if(l4Pose) {
            currentScoringPoint = ScoringPoint.ScoringL4;
        }

        pivotAngle(currentScoringPoint);
        elevator.set(currentScoringPoint);
        
        double pivotSpeed = pivotController.calculate(
            pivotMotor.getEncoder().getPosition(),
            pivotAngle(currentScoringPoint).degrees()
        );

        coralIntake.run();
        pivotMotor.set(pivotSpeed);
        elevator.run();
    }

    /**
     * Overload of run function for autos & stuffs
     */
    public static void run() {
        double pivotSpeed = pivotController.calculate(
            pivotMotor.getEncoder().getPosition(),
            pivotAngle(currentScoringPoint).degrees()
        );

        coralIntake.run();
        pivotMotor.set(pivotSpeed);
        elevator.run();
    }

    public static void _overridePivotSpeed(double speed) {
        pivotMotor.set(speed / 10.0);
    }

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

    public static void recordState() {
        Logger.recordOutput("Coral Intake Speed", coralIntake.get());
        Logger.recordOutput("Coral Intake Current", coralIntake.getCurrent());

        Logger.recordOutput("Coral Pivot Speed", pivotMotor.get());
        Logger.recordOutput("Coral Pivot Set Angle (Degrees)", pivotAngle(currentScoringPoint).degrees());
        Logger.recordOutput("Coral Pivot Setpoint", currentScoringPoint);
        Logger.recordOutput("Coral Pivot Position", pivotMotor.getAbsoluteEncoder().getPosition());

        
        Logger.recordOutput("Elevator Set Point", elevator.getSetpointEnum());
        Logger.recordOutput("Elevator Set Point (Meters)", elevator.getSetpoint());
        Logger.recordOutput("Elevator Position", elevator.getPosition());
        Logger.recordOutput("Elevator At Pose", elevator.atPosition());
        Logger.recordOutput("Elevator Speed", elevator.getSpeed());
    }
}



