package frc.robot.systems;

import java.io.IOException;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.littletonrobotics.junction.Logger;

public class PoseEstimator {
    private static final double AMBIGUITY_CUTOFF = 0.1;
    private static final double DISTANCE_CUTOFF = 5.0;
    
    private static final Transform3d BOT_TO_CAM_FRONT
        = new Transform3d(-0.329, -0.158, 0.201, new Rotation3d(0.0, 0.0, 0));
    private static final Transform3d BOT_TO_CAM_BACK
        = new Transform3d(0.330, -0.214, 0.342, new Rotation3d(0.0, 0.0, Math.PI));

    //private static final PhotonCamera camFront = new PhotonCamera("FrontCam");
    private static final PhotonCamera camBack = new PhotonCamera("BackCam");
    private static final PhotonCamera camTest = new PhotonCamera("CameraTest");

    private static AprilTagFieldLayout field;

   // private static final PhotonPoseEstimator photonEstimatorFront;
    private static final PhotonPoseEstimator photonEstimatorBack;
    private static final PhotonPoseEstimator photonEstimatorTest;
    private static final List<PhotonTrackedTarget> trackedTargets
        = new ArrayList<PhotonTrackedTarget>();

    public static SwerveDrivePoseEstimator3d swervePoseEstimator
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

       /* photonEstimatorFront = new PhotonPoseEstimator(
            field,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            BOT_TO_CAM_FRONT
        );*/

         photonEstimatorBack = new PhotonPoseEstimator(
            field,
            PoseStrategy.LOWEST_AMBIGUITY,
            BOT_TO_CAM_BACK
        );

        photonEstimatorTest = new PhotonPoseEstimator(field, PoseStrategy.LOWEST_AMBIGUITY,BOT_TO_CAM_FRONT);

        //photonEstimatorFront.setReferencePose(new Pose3d());
        photonEstimatorBack.setReferencePose(new Pose3d());
        photonEstimatorTest.setReferencePose(new Pose3d());
    }

    /**
     * Update the estimator with both odometry and vision data.
     */
    public static void update() {
        //Optional<EstimatedRobotPose> estimateFront = updateEstimator(photonEstimatorFront, camBack);
        Optional<EstimatedRobotPose> estimateBack = updateEstimator(photonEstimatorBack, camBack);
        Optional<EstimatedRobotPose> estimateTest = updateEstimator(photonEstimatorTest, camTest);
        //Optional<EstimatedRobotPose> workingEstimate = estimateTest.or(() -> estimateBack);
        
        SwerveDrive.updatePoseEstimator(swervePoseEstimator);

        trackedTargets.clear();
        estimateTest.ifPresent((ef) -> trackedTargets.addAll(ef.targetsUsed));
        //estimateBack.ifPresent((eb) -> trackedTargets.addAll(eb.targetsUsed));
        
        SmartDashboard.putBoolean("VisionTarget", estimateTest.isPresent());
        if(estimateTest.isPresent())
        {
            Robot.visionIsGood=true;
        }
        estimateTest.ifPresent(
            (est) -> swervePoseEstimator
                .addVisionMeasurement(est.estimatedPose, est.timestampSeconds)
        );
        
        
        

        //swervePoseEstimator.addVisionMeasurement(null, AMBIGUITY_CUTOFF);
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
     * Gets an `ArrayList` of the recently tracked targets of the `PoseEstimator`.
     */
    public static ArrayList<Pose3d> trackedTargetPoses() {
        ArrayList<Pose3d> targetPoses = new ArrayList<Pose3d>();
        
        for (PhotonTrackedTarget timedTarget : trackedTargets) {
            field.getTagPose(timedTarget.getFiducialId()).ifPresent((pose) -> targetPoses.add(pose));
        }

        return targetPoses;
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
    public static void reset(Pose2d pose) {
        SwerveDrive.setPoseOf(swervePoseEstimator,
        
        
        new Pose3d(pose.getX(), pose.getY(), 0, new Rotation3d(0, 0, pose.getRotation().getRadians())));
    }

    
     public static void reset(Pose3d pose) {
        SwerveDrive.setPoseOf(swervePoseEstimator, pose);
    }

    public static void reinstantiateDrive() {
        swervePoseEstimator = SwerveDrive
            .createSwervePoseEstimator(
                new Pose3d(
                    0, 0, 0.0,
                    new Rotation3d(
                        Pigeon.getRoll().radians(),
                        Pigeon.getPitch().radians(),
                        Pigeon.getYaw().radians()
                    )
                )
            );
    }

    /**
     * Update an individual estimator with vision data.
     */
    private static Optional<EstimatedRobotPose> updateEstimator(PhotonPoseEstimator estimator, PhotonCamera cam) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        PhotonPipelineResult latestResult = null;

        for (PhotonPipelineResult res : results) {
            if (
                (latestResult == null
                || res.getTimestampSeconds() > latestResult.getTimestampSeconds())
                && res.getBestTarget() != null
                && res.getBestTarget().getPoseAmbiguity() < AMBIGUITY_CUTOFF
                && res.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(new Translation3d()) < DISTANCE_CUTOFF
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
        Logger.recordOutput("Pose Estimator Estimated Robot Pose (3d)", swervePoseEstimator.getEstimatedPosition());
        Logger.recordOutput("Pose Estimator Estimated Robot Pose (2D)", estimate2d());

        Logger.recordOutput(
            "Pose Estimator Tracked Target Poses",
            trackedTargetPoses().toArray(new Pose3d[0])
        );
    }
}
