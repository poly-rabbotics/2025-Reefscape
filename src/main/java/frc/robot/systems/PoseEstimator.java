package frc.robot.systems;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import org.littletonrobotics.junction.Logger;

@Logged
public class PoseEstimator {
    private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private static final Transform3d BOT_TO_CAM_LEFT = new Transform3d(0.305, 0.222, 0.098, new Rotation3d(0.0, 0.0, 0.0));
    private static final Transform3d BOT_TO_CAM_RIGHT = new Transform3d(0.273, 0.222, 0.133, new Rotation3d(0.0, 1.047, 0.0));

    private static final PhotonCamera camRight = new PhotonCamera("RightCam");
    private static final PhotonCamera camLeft = new PhotonCamera("LeftCam");

    private static final PhotonPoseEstimator photonEstimatorLeft
        = new PhotonPoseEstimator(FIELD, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BOT_TO_CAM_LEFT);
    private static final PhotonPoseEstimator photonEstimatorRight
        = new PhotonPoseEstimator(FIELD, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, BOT_TO_CAM_RIGHT);

    private static Optional<EstimatedRobotPose> latestPhotonEstimate = Optional.empty();

    static {
        photonEstimatorLeft.setReferencePose(new Pose3d());
    }

    public static void update() {
        Optional<EstimatedRobotPose> estimateLeft = updateEstimator(photonEstimatorLeft, camLeft);
        Optional<EstimatedRobotPose> estimateRight = updateEstimator(photonEstimatorRight, camRight);
        Optional<EstimatedRobotPose> workingEstimate = estimateLeft.or(() -> estimateRight);

        latestPhotonEstimate = workingEstimate;

        latestPhotonEstimate.ifPresent((est) -> SwerveDrive.setOdometry(est.estimatedPose.toPose2d()));
        Logger.recordOutput("Estimated Robot Pose", estimate());
    }

    /**
     * Returns the current best estimate of pose estimation, being either from photon vision if an
     * april tag is available, or from odometry if it is not.
     */
    public static Pose3d estimate() {
        return latestPhotonEstimate
            .map((est) -> est.estimatedPose)
            .orElse(new Pose3d(SwerveDrive.getOdometryPose()));
    }

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

}
