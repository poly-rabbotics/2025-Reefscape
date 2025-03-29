package frc.robot.systems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;
import frc.robot.subsystems.Coral.CoralIntake.IntakeState;
import frc.robot.systems.Coral.ScoringPoint;

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

            if (stepTimer.get() >= time && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            fn.run();

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
        double measuredAngle =(PoseEstimator.estimate2d().getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;
        double setAngle = (setPathPosition.getRotation().getRadians() % Angle.TAU + Angle.TAU) % Angle.TAU;

        return Math.abs(PoseEstimator.estimate2d().getX() - setPathPosition.getX()) < TOLERANCE_X
            && Math.abs(PoseEstimator.estimate2d().getY() - setPathPosition.getY()) < TOLERANCE_Y
            && Math.abs(measuredAngle - setAngle) < TOLERANCE_THETA;
    }

    public static Function<StepStatus, StepStatus> makeCoralStep(ScoringPoint pose) {
        return (prevState) -> {
            Coral.setScoringPoint(pose);

            if (Coral.atScoringPoint() && prevState == StepStatus.Done) {
                return StepStatus.Done;
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> scoreCoral() {
        final double outtakeTime = 1.5;
        Timer stepTimer = new Timer();

        return (prevState) -> {
            if (!stepTimer.isRunning()) {
                stepTimer.start();
            }

            if (stepTimer.get() >= outtakeTime && prevState == StepStatus.Done) {
                Coral.setIntakeState(IntakeState.Idle);
                return StepStatus.Done;
            }

            if (Coral.getIntakeState() != IntakeState.Outtaking) {
                Coral.setIntakeState(IntakeState.Outtaking);
            }

            return StepStatus.Running;
        };
    }

    private static Function<StepStatus, StepStatus> intakeCoral() {
        return (prevState) -> {
            switch (Coral.getIntakeState()) {
                case Intaking:
                    return StepStatus.Running;

                case Storing:
                    if (prevState == StepStatus.Done) {
                        return StepStatus.Done;
                    }

                    return StepStatus.Waiting;

                default:
                    Coral.setIntakeState(IntakeState.Intaking);
                    return StepStatus.Running;
            }
        };
    }

    private static void buildProcedures() {
        // Does nothing, assumes starting forward.
        procedures[0] = new AutonomousProcedure();

        // Starts center then scores coral far reef right side (driver POV) L3, intakes and moves
        // counter clockwise to score as many L3 as possible. Blue Side.
        procedures[2] = new AutonomousProcedure()

            // Move to right side of far reef (driver POV)
            .andThen(makeDriveStep(new Pose2d(5.75, 3.85, new Rotation2d(Math.PI))))

            // Score coral L3
            .andThen(makeCoralStep(ScoringPoint.ScoringL3))
            .andThen(scoreCoral())

            // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // coral
            .andThen(makeDriveStep(
                new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
                new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            ))
            .andThen(intakeCoral())

            // Move around reef then move to left side of far reef (driver POV)
            .andThen(makeDriveStep(
                new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
                new Pose2d(5.75, 4.18, new Rotation2d(Math.PI))
            ))

            // Score coral L3
            .andThen(makeCoralStep(ScoringPoint.ScoringL3))
            .andThen(scoreCoral())
            
            // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // coral
            .andThen(makeDriveStep(
                new Pose2d(5.75, 5.0, new Rotation2d(Math.PI)),
                new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            ))
            .andThen(intakeCoral())

            // Move to back left reef right/center side
            .andThen(makeDriveStep(new Pose2d(5.26, 5.04, new Rotation2d(-2.1))))

            // Score coral L3
            .andThen(makeCoralStep(ScoringPoint.ScoringL3))
            .andThen(scoreCoral())
            
            // Move to avoid reef and go to the left far intake station (driver POV), then intake a
            // coral
            .andThen(makeDriveStep(
                new Pose2d(1.647, 7.47, new Rotation2d(2.2))
            ))
            .andThen(intakeCoral())

            // Move to back left reef left/away from center side
            .andThen(makeDriveStep(new Pose2d(4.97, 5.22, new Rotation2d(-2.1))))

            // Score coral L3
            .andThen(makeCoralStep(ScoringPoint.ScoringL3))
            .andThen(scoreCoral())
            ;
    }
}
