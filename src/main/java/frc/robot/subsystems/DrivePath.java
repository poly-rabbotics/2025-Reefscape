package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.systems.PoseEstimator;

public class DrivePath {
    private static final double POSE_TOLERANCE = 0.05;
    private static final double ROTATION_TOLERANCE = 2.0;

    private final ArrayList<Pose2d> points;
    private final Function<Pose2d, Boolean> targetReached;

    /**
     * Creates a new drive path with only one position, using a default target reached function.
     */
    public DrivePath(Pose2d pose) {
        targetReached = DrivePath::targetReachedDefault;
        points = new ArrayList<Pose2d>(1);
        points.add(pose);
    }
    
    /**
     * Creates a new drive path with the given positions, in order, using a default target reached
     * function.
     */
    public DrivePath(ArrayList<Pose2d> points) {
        targetReached = DrivePath::targetReachedDefault;
        this.points = points;
    }

    /**
     * Creates a new drive path with only one position, which will be cleared once `targetReached`
     * returns `true`.
     */
    public DrivePath(Pose2d pose, Function<Pose2d, Boolean> targetReached) {
        this.targetReached = targetReached;
        points = new ArrayList<Pose2d>(1);
        points.add(pose);
    }
    
    /**
     * Creates a new drive path with the given positions, in order, advancing when `targetReached`
     * returns true.
     */
    public DrivePath(ArrayList<Pose2d> points, Function<Pose2d, Boolean> targetReached) {
        this.targetReached = targetReached;
        this.points = points;
    }

    /**
     * Gets the current target. The current target position is advanced whenever the `targetReached`
     * function, given in the contructor, returns true.
     */
    public Optional<Pose2d> currentTarget() {
        if (isCompleted()) {
            return Optional.empty();
        }

        return Optional.of(points.get(0));
    }

    /**
     * Returns `true` if all targets have been reached and no more target points remain.
     */
    public boolean isCompleted() {
        if (points.isEmpty()) {
            return true;
        }

        if (targetReached.apply(points.get(0))) {
            points.remove(0);
        }

        return points.isEmpty();
    }

    /**
     * A reasonable default for the `targetReached` function required by drive paths.
     */
    public static boolean targetReachedDefault(Pose2d targetPose) {
        var currentPose = PoseEstimator.estimate2d();

        var distX = currentPose.getX() - targetPose.getX();
        var distY = currentPose.getY() - targetPose.getY();
        var distance = Math.sqrt(distX * distX + distY * distY);

        var arc = currentPose.getRotation().getDegrees() - targetPose.getRotation().getDegrees();

        return distance < POSE_TOLERANCE && arc < ROTATION_TOLERANCE;
    }
}
