package frc.robot.systems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.subsystems.Coral.*;
import frc.robot.subsystems.Coral.CoralIntake.IntakeState;

public class Coral {
    private static final CoralIntake coralIntake = new CoralIntake(11, 7, 1, 2);
    private static final Dealgaeficator algae = new Dealgaeficator(0);
    private static final CoralPivot pivot = new CoralPivot(3);
    private static final Elevator elevator
        = new Elevator(14, InvertedValue.Clockwise_Positive, 0);

    public enum ScoringPoint {
        ScoringL1, ScoringL2, ScoringL3, ScoringL4,
        DealgaeficateL2, DealgaeficateL3, AlgaeNet,
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
                coralIntake.setIntaking();
                break;
        
            case Outtaking:
                coralIntake.setOuttaking();
                break;

            default:
            case Idle:
                coralIntake.setIdle();
                break;
        }
    }

    /**
     * Gets the current state of the intake.
     */
    public static IntakeState getIntakeState() {
        return coralIntake.getState();
    }

    /**
     * Set the desired scoring point for all coral related appendages.
     */
    public static void setScoringPoint(ScoringPoint point) {
        elevator.set(point);
        pivot.pivotTo(point);
    }

    public static void setCoralPosition(double coralPose) {
        coralIntake.setCoralPosition(coralPose);
    }

    public static ScoringPoint getScoringPoint() {
        return elevator.getSetpointEnum();
    }

    /**
     * Whether or not all coral related appendages are at their desired locations.
     */
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
        coralIntake._setSpeed(speed);
    }

    public static void _setElevatorPose(double pose) {
        elevator.setPosition(pose);
    }

    public static void recordState() {
        Logger.recordOutput("Coral Intake Speed", coralIntake.get());
        Logger.recordOutput("Coral Intake Current", coralIntake.getCurrent());
        Logger.recordOutput("Coral Intake State", coralIntake.getState());
        Logger.recordOutput("Coral Intake Storing Coral", coralIntake.isCoralStored());
        Logger.recordOutput("Coral Intake Coral Position", coralIntake.getCoralPosition());
        Logger.recordOutput("Coral Intake Front Prox On", coralIntake.frontProxOn());
        Logger.recordOutput("Coral Intake Back Prox On", coralIntake.backProxOn());

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



