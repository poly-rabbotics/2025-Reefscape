package frc.robot.systems;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.AutonomousProcedure;
import frc.robot.subsystems.DrivePath;
import frc.robot.subsystems.AutonomousProcedure.StepStatus;

public class AutonomousManager {
    private static final int AUTO_SWITCH_COUNT = 4;
    private static AutonomousManager instance = new AutonomousManager();
    private AutonomousProcedure[] procedures = new AutonomousProcedure[1 << AUTO_SWITCH_COUNT];
    private Pose2d[] startingPositions = new Pose2d[1 << AUTO_SWITCH_COUNT];

    private AutonomousManager() {
        // Does nothing, assumes starting forward.
        startingPositions[0] = new Pose2d(0.0, 0.0, new Rotation2d(0));
        procedures[0] = new AutonomousProcedure();
    }

    public static void reset() {
        instance = new AutonomousManager();
    }

    /**
     * Gets an auto procedure by its index.
     */
    public static AutonomousProcedure getAutoProcedure(int mode) {
        try {
            AutonomousProcedure proc = instance.procedures[mode];
            return proc;
        } catch (Exception e) {
            System.out.println("Bad switch value!");            
            return new AutonomousProcedure();
        }
    }

    /**
     * Gets an auto mode by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static AutonomousProcedure getProcedureFromSwitches(boolean...  switches) {
        return getAutoProcedure(getSwitchSelection(switches));
    }

    /**
     * Gets an auto start by its index.
     */
    public static Pose2d getStartingPos(int mode) {
        try {
            Pose2d pose = instance.startingPositions[mode];
            return pose;
        } catch (Exception e) {
            System.out.println("ERROR: Bad switch value");
            return new Pose2d();
        }
    }

    /**
     * Gets an auto start by a set of binary switches, switches should be given
     * in order of least significant bit first.
     */
    public static Pose2d getStartingPosSwitches(boolean...  switches) {
        return getStartingPos(getSwitchSelection(switches));
    }

    public static int getSwitchSelection(boolean...  switches) {
        int mode = 0;

        for (int i = 0; i < switches.length; i++) {
            mode += switches[i] ? (1 << i) : 0;
        }

        return mode;
    }

    public static void log(boolean... switches) {
        Logger.recordOutput("Auto Manager Selection", getSwitchSelection(switches));
        Logger.recordOutput("Auto Manager Modes Length", instance.procedures.length);
    }

    private static Function<StepStatus, StepStatus> makeDriveStep(Pose2d pose) {
        return (prevState) -> {
            DrivePath path = new DrivePath(pose, AutonomousManager::withinPositionTolerance);
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

}
