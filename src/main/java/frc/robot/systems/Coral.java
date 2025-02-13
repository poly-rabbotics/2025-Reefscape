package frc.robot.systems;

import frc.robot.subsystems.Coral.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Angle;

public class Coral {
    private static final double INTAKE_SPEED = -0.75;
    private static final double OUTTAKE_SPEED = 0.75;

    private static final Angle PIVOT_POINT_STOW = new Angle().setDegrees(110.0);
    private static final Angle PIVOT_POINT_INTAKE = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L1 = new Angle().setDegrees(210.0);
    private static final Angle PIVOT_POINT_L2 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L3 = new Angle().setDegrees(0.0);
    private static final Angle PIVOT_POINT_L4 = new Angle().setDegrees(0.0);

    private static final double PIVOT_PID_P = 0.0;
    private static final double PIVOT_PID_I = 0.0;
    private static final double PIVOT_PID_D = 0.0;

    private static final SparkMax pivotMotor = new SparkMax(5, MotorType.kBrushless);

    private static CoralIntake coralIntake;
    private static IntakeState intakeState = IntakeState.IDLE;

    private static final PIDController pivotController
        = new PIDController(PIVOT_PID_P, PIVOT_PID_I, PIVOT_PID_D);

    private static IntakePivotPoint pivotSetPoint = IntakePivotPoint.STOW;
    private static Angle pivotSetAngle = PIVOT_POINT_STOW;

    public enum IntakeState {
        INTAKING, OUTTAKING, IDLE
    }

    public enum IntakePivotPoint {
        L1, L2, L3, L4, STOW, INTAKE
    }

    static {
        SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig pivotEncoderConfig = new AbsoluteEncoderConfig();

        pivotEncoderConfig.positionConversionFactor(360.0);
        pivotMotorConfig.apply(pivotEncoderConfig);
        pivotMotor.configure(
            pivotMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        coralIntake = new CoralIntake(7, 11);
    }

    public static void setIntakeState(IntakeState state) {
    }

    public static void pivotTo(IntakePivotPoint setPivot) {
        pivotSetPoint = setPivot;
        pivotSetAngle = pivotAngle(setPivot);
    }

    public static void run(boolean intake, boolean outtake, boolean stowedPose, boolean coralStationPose, boolean level1Pose, boolean level2Pose, boolean level3Pose, boolean level4Pose) {
        // SET PIVOT POINT
        if(stowedPose) {
            pivotTo(IntakePivotPoint.STOW);
        } else if(coralStationPose) {
            pivotTo(IntakePivotPoint.INTAKE);
        } else if(level1Pose) {
            pivotTo(IntakePivotPoint.L1);
        }  else if(level2Pose) {
            pivotTo(IntakePivotPoint.L2);
        }  else if(level3Pose) {
            pivotTo(IntakePivotPoint.L3);
        }  else if(level4Pose) {
            pivotTo(IntakePivotPoint.L4);
        } 

        // SET INTAKE MODE
        if(intake) {
            intakeState = IntakeState.INTAKING;
        } else if(outtake) {
            intakeState = IntakeState.OUTTAKING;
        } else {
            intakeState = IntakeState.IDLE;
        }

        // SET INTAKE SPEED
        if (intakeState == IntakeState.INTAKING) {
            coralIntake.set(INTAKE_SPEED);
        } else if (intakeState == IntakeState.OUTTAKING) {
            coralIntake.set(OUTTAKE_SPEED);
        } else {
            coralIntake.set(0);
        }
        
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
            case L1:
                return PIVOT_POINT_L1;
            case L2:
                return PIVOT_POINT_L2;
            case L3:
                return PIVOT_POINT_L3;
            case L4:
                return PIVOT_POINT_L4;
            case STOW:
                return PIVOT_POINT_STOW;
            case INTAKE:
                return PIVOT_POINT_INTAKE;
            default:
                return PIVOT_POINT_STOW;
        }
    }

    public static void recordState() {
        Logger.recordOutput("Coral Intake Pivot Speed", pivotMotor.get());
        Logger.recordOutput("Coral Intake Set Angle (Degrees)", pivotSetAngle.degrees());
        Logger.recordOutput("Coral Intake Set Point", pivotSetPoint);
        Logger.recordOutput("Coral Intake Speed", coralIntake.get());
        Logger.recordOutput("Coral Intake Pivot Position", pivotMotor.getAbsoluteEncoder().getPosition());
    }
}



