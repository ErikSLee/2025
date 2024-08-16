package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PhotonVisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class Photonvision extends SubsystemBase {
    static boolean enablePhotonInstances = true; // Nyahaha
    private final PhotonCamera camera;
    private final PhotonCamera camera2;
    
    private PhotonPipelineResult pipelineResult;
    AprilTagFieldLayout aprilTagFieldLayout = null;
    PhotonPoseEstimator photonPoseEstimator;
    private static final List<Integer> speakerCenterTargets = Arrays.asList(4, 7);
    PhotonPoseEstimator photonPoseEstimator2;

    /**
     *  Using Photonvision to update the robot odometry based on april tag locations. Example from 
     * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
     * 
     */
    public Photonvision() {
        camera = new PhotonCamera(PhotonVisionConstants.kCameraName);
        camera2 = new PhotonCamera(PhotonVisionConstants.kCameraName2);
        
        pipelineResult = new PhotonPipelineResult();
        /**
         * Vision setup
         */
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            /*
             * The camera relative to the robot
             */
            Transform3d robotToCam = new Transform3d(
                    new Translation3d(Units.inchesToMeters(10), 0, Units.inchesToMeters(12)),
                    new Rotation3d(0, Math.toRadians(16.0),
                            Math.PI));
            photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    this.getCamera(),
                    robotToCam);
            photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    this.getCamera2(),
                    robotToCam);

        } catch (IOException e) {
            DriverStation.reportError(e.toString(), true);
        }
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public PhotonCamera getCamera2() {
        return camera2;
    }

    public boolean hasTargets() {
        return pipelineResult.hasTargets();
    }

    public List<PhotonTrackedTarget> targets() {
        return pipelineResult.targets;
    }

    public PhotonTrackedTarget getBestTarget() {
        return pipelineResult.getBestTarget();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public PhotonPipelineResult getLatestResults() {
        return pipelineResult;
    }

    @Override
    public void periodic() {
        pipelineResult = camera.getLatestResult();
        SmartDashboard.putBoolean("CameraTargets", pipelineResult.hasTargets());
        if (pipelineResult.hasTargets()) {
            SmartDashboard.putNumber("DegreesToSpeaker", getSpeakerTarget());

        }

    }

    public static void enableVision(boolean enable) {
        enablePhotonInstances = enable;
    }

    public boolean hasSpeakerTarget() {
        if (pipelineResult.hasTargets()) {
            if (speakerCenterTargets.contains(pipelineResult.getBestTarget().getFiducialId())) {
                return true;
            }
        }
        return false;
    }

    /**
     * 
     * @return RADIANs to speaker target
     */
    public double getSpeakerTarget() {
        if (pipelineResult.hasTargets()) {
            if (speakerCenterTargets.contains(pipelineResult.getBestTarget().getFiducialId())) {
                return Math.toRadians(pipelineResult.getBestTarget().getYaw());
            }
        }
        return -999;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     * as taken from: https://github.com/PhotonVision/photonvision/blob/master/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
     *
     *  @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = PhotonVisionConstants.kSingleTagStdDevs;
        var targets = getLatestResults().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = PhotonVisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

}