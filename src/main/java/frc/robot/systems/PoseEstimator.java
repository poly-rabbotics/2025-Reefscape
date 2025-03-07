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
    private static final Transform3d BOT_TO_CAM_FRONT
        = new Transform3d(-0.329, 0.158, 0.201, new Rotation3d(0.0, 0.0, 150.0 * Math.PI / 180.0));
    private static final Transform3d BOT_TO_CAM_BACK
        = new Transform3d(0.330, 0.214, 0.342, new Rotation3d(0.0, 0.0, 30.0 * Math.PI / 180.0));

    private static final PhotonCamera camFront = new PhotonCamera("FrontCam");
    private static final PhotonCamera camBack = new PhotonCamera("BackCam");

    private static AprilTagFieldLayout field;
    private static final PhotonPoseEstimator photonEstimatorFront;
    private static final PhotonPoseEstimator photonEstimatorBack;

    private static SwerveDrivePoseEstimator3d swervePoseEstimator
        = SwerveDrive.createSwervePoseEstimator(new Pose3d());

    static {
        try {
            field = new AprilTagFieldLayout(
                Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape.json")
            );
        } catch (IOException e) {
            System.out.println("ERROR: Failed to load field JSON, falling back on default");
            field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        }

        photonEstimatorFront = new PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            BOT_TO_CAM_FRONT
        );

        photonEstimatorBack = new PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            BOT_TO_CAM_BACK
        );

        photonEstimatorFront.setReferencePose(new Pose3d());
        photonEstimatorBack.setReferencePose(new Pose3d());
    }

    /**
     * Update the estimator with both odometry and vision data.
     */
    public static void update() {
        Optional<EstimatedRobotPose> estimateFront = updateEstimator(photonEstimatorFront, camBack);
        Optional<EstimatedRobotPose> estimateBack = updateEstimator(photonEstimatorBack, camFront);
        Optional<EstimatedRobotPose> workingEstimate = estimateFront.or(() -> estimateBack);

        SwerveDrive.updatePoseEstimator(swervePoseEstimator);
        workingEstimate.ifPresent(
            (est) -> swervePoseEstimator
                .addVisionMeasurement(est.estimatedPose, est.timestampSeconds)
        );
    }

    /**
     * Returns the current best estimate of pose estimation.
     */
    public static Pose3d estimate() {
        Pose3d estimate = swervePoseEstimator.getEstimatedPosition();

        return new Pose3d(
            estimate.getX(),
            estimate.getY(),
            estimate.getZ(),
            new Rotation3d(
                Pigeon.getRoll().radians(),
                Pigeon.getPitch().radians(),
                Pigeon.getYaw().radians()
            )
        );
    }

    /**
     * Returns the current best estimate of pose estimation. Projects this position onto the XY
     * plane.
     */
    public static Pose2d estimate2d() {
        return estimate().toPose2d();
    }

    /**
     * Zero position. This will quickly be undone by a vision target, but until then odometry will
     * start at the origin.
     */
    public static void reset() {
        SwerveDrive.setPoseOf(swervePoseEstimator, new Pose3d());
    }

    /**
     * Set position. This will quickly be undone by a vision target, but until then odometry will
     * start at the origin.
     */
    public static void reset(Pose3d pose) {
        SwerveDrive.setPoseOf(swervePoseEstimator, pose);
    }

    /**
     * Update an individual estimator with vision data.
     */
    private static Optional<EstimatedRobotPose> updateEstimator(PhotonPoseEstimator estimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        PhotonPipelineResult latestResult = null;

        for (PhotonPipelineResult res : results) {
            if (
                latestResult == null
                || res.getTimestampSeconds() > latestResult.getTimestampSeconds()
            ) {
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
