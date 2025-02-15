package frc.robot.systems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.systems.CoralIntake.IntakePivotPoint;
import frc.robot.systems.CoralIntake.IntakeState;
import frc.robot.systems.Elevator.ElevatorPose;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousProcedure[] procedures = new AutonomousProcedure[1 << AUTO_SWITCH_COUNT];

    static {
        buildProcedures();
    }

    public static void reset() {
        buildProcedures();
    }

    /**
     * Gets an auto procedure by its index.
     */
    public static Optional<AutonomousProcedure> getAutoProcedure(int mode) {
        if (mode < 0 || mode > procedures.length) {
            return Optional.empty();
        }

        return Optional.of(procedures[mode]);
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static Optional<AutonomousProcedure> getProcedureFromSwitches(boolean... switches) {
        return getAutoProcedure(getSwitchSelection(switches));
    }

    public static int getSwitchSelection(boolean... switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        return mode;
    }

    private static Function<StepStatus, StepStatus> makeStep(Runnable fn) {
        return (prevState) -> {
            fn.run();
            
            if (prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> makeTimedStep(double time) {
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }
    private static Function<StepStatus, StepStatus> makeTimedStep(double time, Runnable fn) {
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            fn.run();

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    /**
     * Makes a step for running the drive.
     */
    private static Function<StepStatus, StepStatus> makeDriveStep(Pose2d... poses) {
        ArrayList<Pose2d> listPoses = new ArrayList<Pose2d>();

        for (Pose2d pose : poses) {
            listPoses.add(pose);
        }
        
        DrivePath path = new DrivePath(listPoses, AutonomousManager::withinPositionTolerance);
        return makeDriveStep(path);
    }

    /**
     * Makes a step for running the drive.
     */
    private static Function<StepStatus, StepStatus> makeDriveStep(DrivePath path) {
        return (prevState) -> {
            SwerveDrive.setPath(path);

            if (path.isCompleted() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    /**
     * Returns true if the drive is within tolerance to its desired position. Intended for use with
     * a swerve mode which takes over driving.
     */
    public static boolean withinPositionTolerance(Pose2d setPathPosition) {
        final double TOLERANCE_X = 0.3;
        final double TOLERANCE_Y = 0.3;
        final double TOLERANCE_THETA = 0.1;

        // makes sure angle are within [0, tau)
        double measuredAngle = (PoseEstimator.estimate2d().getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;
        double setAngle = (setPathPosition.getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;

        return Math.abs(PoseEstimator.estimate2d().getX() - setPathPosition.getX()) < TOLERANCE_X
            && Math.abs(PoseEstimator.estimate2d().getY() - setPathPosition.getY()) < TOLERANCE_Y
            && Math.abs(measuredAngle - setAngle) < TOLERANCE_THETA;
    }

    public static Function<StepStatus, StepStatus> makeElevatorStep(ElevatorPose pose) {
        return (prevState) -> {
            Elevator.setPose(pose);

            if (Elevator.atPose() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    public static Function<StepStatus, StepStatus> makeIntakePivotStep(IntakePivotPoint point) {
        return (prevState) -> {
            CoralIntake.pivotTo(point);

            if (CoralIntake.atPivotPoint() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static void buildProcedures() {
        // Does nothing, assumes starting forward.
        procedures[0] = new AutonomousProcedure();

        procedures[1] = new AutonomousProcedure()
            .andThen(makeDriveStep(new Pose2d(12.5, 5.0, new Rotation2d(-60.0 / 180.0 * Math.PI))))
            .and(makeElevatorStep(ElevatorPose.ScoringL4))
            .and(makeIntakePivotStep(IntakePivotPoint.ScoringL4))
            .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntakeState(IntakeState.Outtaking)))
            .andThen(makeStep(() -> CoralIntake.setIntakeState(IntakeState.Idle)))
            .and(makeElevatorStep(ElevatorPose.Intaking))
            .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
            .and(makeDriveStep(new Pose2d(16.0, 7.4, new Rotation2d(54.0 / 180.0 * Math.PI))));

        procedures[2] = new AutonomousProcedure()    
            .andThen(makeDriveStep(new Pose2d(14.3,4, new Rotation2d(180.0 / 180.0 * Math.PI))))
            .and(makeElevatorStep(ElevatorPose.ScoringL2))
            .and(makeIntakePivotStep(IntakePivotPoint.ScoringL2))
            .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntakeState(IntakeState.Outtaking)))
            .andThen(makeStep(() -> CoralIntake.setIntakeState(IntakeState.Intaking)))
            .and(makeElevatorStep(ElevatorPose.Intaking))
            .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
            .andThen(makeDriveStep(new Pose2d(16.0, 0.6, new Rotation2d(-54 / 180 * Math.PI))))
            .andThen(makeTimedStep(5.0))
            .andThen(makeDriveStep(new Pose2d(13.7, 3, new Rotation2d(120.0/ 180.0 * Math.Pi))));
            
        procedures[3] = new AutonomousProcedure()
            .andThen(makeDriveStep(new Pose2d(12.5, 3, new Rotation2d(60 / 180 * Math.PI))))
            .and(makeElevatorStep(ElevatorPose.ScoringL4))
            .and(makeIntakePivotStep(IntakePivotPoint.ScoringL4))
            .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntakeState(IntakeState.Outtaking)))
            .andThen(makeStep( ()-> CoralIntake.SetIntakeState(IntakeState.Idle)))
            .and(makeElevatorStep(ElevatorPose.Stowed))
            .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
            .and(makeDriveStep(new Pose2d(14.3, 4, new Rotation2d(180 / 180 * Math.PI))));

        procedures[4] = new AutonomousProcedure()
           .andThen(makeDriveStep(new Pose2d(12, 4, new Rotation2d(0 / 180 * Math.PI))))
           .and(makeElevatorStep(ElevatorPose.ScoringL2));
           .and(makeIntakePivotStep(IntakePivotPoint.ScoringL2))
           .andThen(maketimedStep(3.0, () -> CoralIntake.setIntake(IntakeState.Outtaking)))
           .andThen(makestep(() -> CoralIntake.setIntake(IntakeState.Intaking)))
           .and(makeElevatorStep(ElevatorPose.Intaking))
           .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
           .and(makeDriveStep(new Pose2d(16, 7.4, new Rotation2d( 54 / 180 * Math.PI))))
           .andThen(makeTimedStep(5.0))
           .andThen(makestep(() -> CoralIntake.setIntake(IntakeState.Idle)))
           .and(makeDriveStep(new Pose2d(13.7, 5.1, new Rotation2d( -120 / 180 * Math.PI))))
           .and(makeElevatorStep(ElevatorPose.ScoringL2))
           .and(makeIntakePivotStep(IntakePivotPoint.ScoringL2))
           .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntakeState(IntakeState.Outtaking)))
           .andThen(makeElevatorStep(ElevatorPose.Stowed))
           .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
           .and(makeStep( ()-> CoralIntake.SetIntakeState(IntakeState.Idle)));

        procedures[5] = new AutonomousProcedure()
           .andThen(makeDriveStep(new Pose2d(13.7, 3, new Rotation2d(120 / 180 * Math.PI))))
           .and(makeElevatorStep(ElevatorPose.ScoringL3))
           .and(makeIntakePivotStep(IntakePivotPoint.ScoringL3))
           .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntake(IntakeState.Outtaking)))
           .andThen(makeElevatorStep(ElevatorPose.intaking))
           .and(makeIntakePivotStep(IntakePivotPoint.intaking))
           .and(makestep(() -> CoralIntake.setIntake(IntakeState.Intaking)))
           .and(makeDriveStep(new Pose2d(16, 0.6, new Rotation2d( -54 / 180 * Math.Pi))))
           .and(makeTimedStep(5.0))
           .andThen(makeDriveStep(new Pose2d(13.7, 3, new Rotation2d( 120 / 180 * Math.Pi))))
           .and(makeElevatorStep(ElevatorPose.ScoringL2))
           .and(makeIntakePivotStep(IntakePivotPoint.ScoringL2))
           .andThen(makeTimedStep(3.0, () -> CoralIntake.setIntake(IntakeState.Outtaking)))
           .andThen(makeElevatorStep(ElevatorPose.Stowed))
           .and(makeIntakePivotStep(IntakePivotPoint.Intaking))
           .and(makeStep( ()-> CoralIntake.SetIntakeState(IntakeState.Idle)));

        
    }
}
