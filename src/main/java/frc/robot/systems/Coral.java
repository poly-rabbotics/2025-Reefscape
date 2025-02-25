package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.subsystems.Coral.*;

public class Coral {
    private static final double INTAKE_SPEED = -0.75;
    private static final double OUTTAKE_SPEED = 0.75;

    private static final CoralIntake coralIntake = new CoralIntake(7, 11);
    private static final CoralPivot pivot = new CoralPivot(5);
    private static final Elevator elevator = new Elevator(14, InvertedValue.Clockwise_Positive);
    private static final CoralIntakesLittleBrother algae = new CoralIntakesLittleBrother(0);

    private static ScoringPoint currentScoringPoint = ScoringPoint.Stowed;

    public enum IntakeState {
        Intaking, Outtaking, Idle
    }

    public enum ScoringPoint {
        ScoringL1, ScoringL2, ScoringL3, ScoringL4, Stowed, Intaking
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
    public static void run(boolean intaking, boolean outtaking, boolean stowedPose, boolean intakePose, boolean l1Pose, boolean l2Pose, boolean l3Pose, boolean l4Pose, double algaeSpeed, boolean isAlgae) {
        // Coral intake
        if (intaking) {
            setIntakeState(IntakeState.Intaking);
        } else if (outtaking) {
            setIntakeState(IntakeState.Outtaking);
        } else {
            setIntakeState(IntakeState.Idle);
        }

        if (stowedPose) {
            currentScoringPoint = ScoringPoint.Stowed;
        } else if (intakePose) {
            currentScoringPoint = ScoringPoint.Intaking;
        } else if (l1Pose) {
            currentScoringPoint = ScoringPoint.ScoringL1;
        } else if (l2Pose) {
            currentScoringPoint = ScoringPoint.ScoringL2;
        } else if (l3Pose) {
            currentScoringPoint = ScoringPoint.ScoringL3;
        } else if (l4Pose) {
            currentScoringPoint = ScoringPoint.ScoringL4;
        }

        elevator.set(currentScoringPoint);
        pivot.pivotTo(currentScoringPoint);
        algae.run(algaeSpeed);
        
        run(isAlgae);
    }

    /**
     * Overload of run function for autos & stuffs
     */
    public static void run(boolean isAlgae) {
        pivot.run(isAlgae);
        coralIntake.run();
        elevator.run(isAlgae);
    }

    public static void setIntakeState(IntakeState state) {
        switch (state) {
            case Intaking:
                coralIntake.set(INTAKE_SPEED);
                break;

            case Outtaking:
                coralIntake.set(OUTTAKE_SPEED);
                break;

            case Idle:
            default:
                coralIntake.set(0);
                break;
        }
    }

    public static void setScoringPoint(ScoringPoint point) {
        currentScoringPoint = point;
    }

    public static boolean atScoringPoint() {
        return elevator.atPosition() && pivot.atPivotPoint();
    }

    public static void _setPivotSpeed(double pivotSpeed) {
        pivot._setSpeed(pivotSpeed);
    }

    public static void _setElevatorSpeed(double elevatorSpeed) {
        elevator._setSpeed(elevatorSpeed);
    }

    public static void _setElevatorPose(double pose) {
        elevator.setPosition(pose);
    }

    public static void recordState() {
        Logger.recordOutput("Coral Intake Speed", coralIntake.get());
        Logger.recordOutput("Coral Intake Current", coralIntake.getCurrent());

        Logger.recordOutput("Coral Pivot Speed", pivot.get());
        Logger.recordOutput("Coral Pivot Set Angle (Degrees)", pivot.getSetAngle().degrees());
        Logger.recordOutput("Coral Pivot Position (Degrees)", pivot.getAngle().degrees());
        Logger.recordOutput("Coral Pivot Setpoint", pivot.getSetPoint());
        Logger.recordOutput("Coral Pivot Velocity", pivot.getVelocity());

        
        Logger.recordOutput("Elevator Set Point", elevator.getSetpointEnum());
        Logger.recordOutput("Elevator Set Point (Meters)", elevator.getSetpoint());
        Logger.recordOutput("Elevator Position", elevator.getPosition());
        Logger.recordOutput("Elevator At Pose", elevator.atPosition());
        Logger.recordOutput("Elevator Speed", elevator.getSpeed());
        Logger.recordOutput("Elevator Velocity", elevator.getVelocity());
    }
}



