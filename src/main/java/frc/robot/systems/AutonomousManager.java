package frc.robot.systems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;

import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

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

    public static void log(boolean... switches) {
        Logger.recordOutput("Auto Manager Selection", getSwitchSelection(switches));
        Logger.recordOutput("Auto Manager Modes Length", procedures.length);
    }

    private static Function<StepStatus, StepStatus> makeDriveStep(Pose2d... poses) {
        ArrayList<Pose2d> listPoses = new ArrayList<Pose2d>();

        for (Pose2d pose : poses) {
            listPoses.add(pose);
        }
        
        DrivePath path = new DrivePath(listPoses, AutonomousManager::withinPositionTolerance);
        return makeDriveStep(path);
    }

    private static Function<StepStatus, StepStatus> makeDriveStep(DrivePath path) {
        return (prevState) -> {
            SwerveDrive.setPath(path);

            if (path.isCompleted()) {
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

    private static void buildProcedures() {
        // Does nothing, assumes starting forward.
        procedures[0] = new AutonomousProcedure();
    }
}
