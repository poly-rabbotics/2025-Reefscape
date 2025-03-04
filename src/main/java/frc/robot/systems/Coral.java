package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.subsystems.Coral.*;

public class Coral {
    private static final double INTAKE_SPEED = -0.75;
    private static final double OUTTAKE_SPEED = 0.75;

    private static final CoralIntake coralIntake = new CoralIntake(7, 11);
    private static final CoralPivot pivot = new CoralPivot(3);
    private static final Elevator elevator = new Elevator(14, InvertedValue.Clockwise_Positive, 0, 1, 2);
    private static final Dealgaeficator algae = new Dealgaeficator(0);

    public enum IntakeState {
        Intaking, Outtaking, Idle
    }

    public enum ScoringPoint {
        ScoringL1, ScoringL2, ScoringL3, ScoringL4,
        DealgaeficateL2, DealgaeficateL3,
        Stowed, Intaking, Handoff
    }

    /**
     * Overload of run function for autos & stuffs
     */
    public static void run() {
        pivot.run();
        coralIntake.run();
        elevator.run();
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
        elevator.set(point);
        pivot.pivotTo(point);
    }

    public static boolean atScoringPoint() {
        return elevator.atPosition() && pivot.atPivotPoint();
    }

    public static void setDealgaeficatorSpeed(double speed) {
        algae.set(speed);
    }

    public static void _setPivotSpeed(double pivotSpeed) {
        pivot._setSpeed(pivotSpeed);
    }

    public static void _setElevatorSpeed(double elevatorSpeed) {
        elevator._setSpeed(elevatorSpeed);
    }

    public static void _setIntakeSpeed(double speed) {
        coralIntake.set(speed);
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
        Logger.recordOutput("Elevator Set Point (in)", elevator.getSetpoint());
        Logger.recordOutput("Elevator Position (Top Encoder) (in)", elevator.getPosition());
        Logger.recordOutput("Elevator Position (in)", elevator.getKrakenPosition());
        Logger.recordOutput("Elevator At Pose", elevator.atPosition());
        Logger.recordOutput("Elevator Speed", elevator.getSpeed());
        Logger.recordOutput("Elevator Velocity (in per second)", elevator.getVelocity());
        Logger.recordOutput("Elevator Acceleration (in per second^2)", elevator.getAcceleration());
        Logger.recordOutput("Elevator At Lower Limit", elevator.atLowerLimit());

        Logger.recordOutput("Algae Intake Speed", algae.get());
    }
}



