package frc.robot.systems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import org.littletonrobotics.junction.Logger;

public class PoseEstimator {
    private static final Transform3d BOT_TO_CAM_LEFT = new Transform3d(0.305, 0.222, 0.098, new Rotation3d(0.0, 0.0, 0.0));
    private static final Transform3d BOT_TO_CAM_RIGHT = new Transform3d(0.273, 0.222, 0.133, new Rotation3d(0.0, 1.047, 0.0));

    private static final PhotonCamera camRight = new PhotonCamera("RightCam");
    private static final PhotonCamera camLeft = new PhotonCamera("LeftCam");

    private static AprilTagFieldLayout field;
    private static final PhotonPoseEstimator photonEstimatorLeft;
    private static final PhotonPoseEstimator photonEstimatorRight;

    private static SwerveDrivePoseEstimator3d swervePoseEstimator = SwerveDrive.createSwervePoseEstimator(new Pose3d());

    static {
        try {
            field = new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json"));
        } catch (IOException e) {
            System.out.println("ERROR: Failed to load field JSON, falling back on default");
            field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        }

        photonEstimatorLeft = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BOT_TO_CAM_LEFT);
        photonEstimatorRight = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BOT_TO_CAM_RIGHT);
        photonEstimatorLeft.setReferencePose(new Pose3d());
        photonEstimatorRight.setReferencePose(new Pose3d());
    }

    /**
     * Update the estimator with both odometry and vision data.
     */
    public static void update() {
        Optional<EstimatedRobotPose> estimateLeft = updateEstimator(photonEstimatorLeft, camLeft);
        Optional<EstimatedRobotPose> estimateRight = updateEstimator(photonEstimatorRight, camRight);
        Optional<EstimatedRobotPose> workingEstimate = estimateLeft.or(() -> estimateRight);

        SwerveDrive.updatePoseEstimator(swervePoseEstimator);
        workingEstimate.ifPresent((est) -> swervePoseEstimator.addVisionMeasurement(est.estimatedPose, est.timestampSeconds));
    }

    /**
     * Returns the current best estimate of pose estimation, being either from photon vision if an
     * april tag is available, or from odometry if it is not.
     */
    public static Pose3d estimate() {
        return swervePoseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current best estimate of pose estimation, being either from photon vision if an
     * april tag is available, or from odometry if it is not. Projects this position onto the XY
     * plane.
     */
    public static Pose2d estimate2d() {
        return swervePoseEstimator.getEstimatedPosition().toPose2d();
    }

    /**
     * Zero position. This will quickly be undone by a vision target, but until then odometry will
     * start at the origin.
     */
    public static void reset() {
        swervePoseEstimator.resetPose(new Pose3d());
    }

    /**
     * Update an individual estimator with vision data.
     */
    private static Optional<EstimatedRobotPose> updateEstimator(PhotonPoseEstimator estimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        PhotonPipelineResult latestResult = null;

        for (PhotonPipelineResult res : results) {
            if (latestResult == null || res.getTimestampSeconds() > latestResult.getTimestampSeconds()) {
                latestResult = res;
            }
        }

        if (latestResult == null) {
            return Optional.empty();
        }

        return estimator.update(latestResult);
    }

    public static void recordState() {
        Logger.recordOutput("Estimated Robot Pose", estimate());
        Logger.recordOutput("Estimated Robot Pose (2D)", estimate2d());
    }
}
