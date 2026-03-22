package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(4.5, 0, 0.25);

    public Swerve() {
        super(
                TunerConstants.DrivetrainConstants,
                0,
                VecBuilder.fill(0.1, 0.1, 0.1),
                VecBuilder.fill(0.1, 0.1, 0.1),
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        );

        setOperatorPerspectiveForward(kBlueAlliancePerspectiveRotation);
    }

    // -------------------------------------------------------------------------
    // Gyro helpers (replaces Drive.java's GyroIO)
    // -------------------------------------------------------------------------

    /** Returns the current robot heading from the CTRE Pigeon2. */
    public Rotation2d getRawGyroRotation() {
        return getState().Pose.getRotation();
    }

    /** Returns yaw angular velocity in radians per second. */
    public double getYawVelocityRadPerSec() {
        return Units.degreesToRadians(
                getPigeon2().getAngularVelocityZWorld().getValueAsDouble()
        );
    }

    // -------------------------------------------------------------------------
    // Pose helpers
    // -------------------------------------------------------------------------

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void setPose(Pose2d pose) {
        resetPose(pose);
    }

    // -------------------------------------------------------------------------
    // AutoFactory / path following
    // -------------------------------------------------------------------------

    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
                this::getPose,
                this::resetPose,
                this::followPath,
                true,
                this,
                trajLogger
        );
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getPose();

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading
        );
        targetSpeeds.omegaRadiansPerSecond = MathUtil.clamp(
                targetSpeeds.omegaRadiansPerSecond, -3.0, 3.0
        );

        setControl(
                pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    // -------------------------------------------------------------------------
    // Vision pipeline (migrated from Drive.java)
    // -------------------------------------------------------------------------

    private void visionPipeline() {
        Rotation2d rawGyroRotation = getRawGyroRotation();
        double yawVelRadPerSec = getYawVelocityRadPerSec();

        // Update Limelight orientation
        LimelightHelpers.SetRobotOrientation("limelight-left",  rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-right", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);

        // Get measurements
        LimelightHelpers.PoseEstimate mt2_left  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        LimelightHelpers.PoseEstimate mt2_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

        LimelightHelpers.PoseEstimate mt1_left  = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        LimelightHelpers.PoseEstimate mt1_right = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        // ----------------- Stage 1: Hard Reject -----------------
        boolean reject = false;

        if (mt2_left == null || mt2_right == null) return;
        if (mt1_left == null || mt1_right == null) return;
        if (mt2_left.tagCount < 1 && mt2_right.tagCount < 1) reject = true;

        // Reject if spinning too fast
        if (Math.abs(yawVelRadPerSec) > Math.toRadians(120)) reject = true;

        if (!reject) {
            // ----------------- Combine Measurements -----------------
            Pose2d avgPose = averagePoseXY(mt1_left.pose, mt1_right.pose, rawGyroRotation);
            avgPose = averagePoseRot(avgPose, mt2_left.pose, mt2_right.pose);

            int tagCount    = Math.max(mt2_left.tagCount, mt2_right.tagCount);
            double distance = Math.min(mt2_left.avgTagDist, mt2_right.avgTagDist);

            // ----------------- Stage 2: Compute Metrics -----------------
            double angleDiff = rawGyroRotation.minus(avgPose.getRotation()).getDegrees();

            // ----------------- Stage 3: Score -----------------
            double score = 1.0;

            // Tag count weight
            score *= Math.min(tagCount / 2.0, 1.0);

            // Distance weight (6m = low trust)
            score *= Math.max(0.0, 1.0 - (distance / 6.0));

            // Angle agreement weight
            score *= Math.max(0.0, 1.0 - (Math.abs(angleDiff) / 30.0));

            // ----------------- Stage 4: Convert to Std Devs -----------------
            double xyStdDev    = 0.3 + (1.5 * (1.0 - score));
            double thetaStdDev = Units.degreesToRadians(10 + (60 * (1.0 - score)));

            setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

            // ----------------- Stage 5: Acceptance -----------------
            boolean accept = false;

            if (score > 0.7) {
                accept = true;
            } else if (score > 0.4 && Math.abs(angleDiff) < 45) {
                accept = true;
            }

            // ----------------- Stage 6: Apply -----------------
            if (accept) {
                super.addVisionMeasurement(
                        avgPose,
                        Utils.fpgaToCurrentTime(mt2_right.timestampSeconds)
                );
            }

            // ----------------- Stage 7: Recovery Mode -----------------
            boolean multiTagStable =
                    tagCount >= 3 &&
                            Math.abs(angleDiff) < 5 &&
                            Math.abs(yawVelRadPerSec) < Math.toRadians(30);

            SmartDashboard.putBoolean("Updating from multitags", multiTagStable);
            if (multiTagStable) {
                resetPose(avgPose);
            }

            // ----------------- Debug -----------------
            SmartDashboard.putNumber("VisionScore",           score);
            SmartDashboard.putNumber("VisionAngleDiff",       angleDiff);
            SmartDashboard.putNumber("VisionXYStdDev",        xyStdDev);
            SmartDashboard.putNumber("VisionThetaStdDevDeg",  Math.toDegrees(thetaStdDev));
            SmartDashboard.putNumber("mt2_left.avgTagDist",   mt2_left.avgTagDist);
            SmartDashboard.putNumber("mt2_right.avgTagDist",  mt2_right.avgTagDist);
            SmartDashboard.putNumber("pose x",     getPose().getX());
            SmartDashboard.putNumber("pose y",     getPose().getY());
            SmartDashboard.putNumber("pose rot",   getPose().getRotation().getDegrees());
            SmartDashboard.putNumber("vision x",   avgPose.getX());
            SmartDashboard.putNumber("vision y",   avgPose.getY());
            SmartDashboard.putNumber("avgPose rot", avgPose.getRotation().getDegrees());
            SmartDashboard.putNumber("diff angle", angleDiff);
        }

        SmartDashboard.putNumber("estimated x",    getPose().getX());
        SmartDashboard.putNumber("estimated y",    getPose().getY());
        SmartDashboard.putNumber("rawGyro",        rawGyroRotation.getDegrees());
        SmartDashboard.putNumber("odometry rotation", getPose().getRotation().getDegrees());
    }

    // -------------------------------------------------------------------------
    // Vision pose averaging helpers (migrated from Drive.java)
    // -------------------------------------------------------------------------

    private Pose2d averagePoseXY(Pose2d a, Pose2d b, Rotation2d rawGyroRotation) {
        double avgX = (a.getX() + b.getX()) / 2.0;
        double avgY = (a.getY() + b.getY()) / 2.0;

        SmartDashboard.putNumber("left_rot",  a.getRotation().getDegrees());
        SmartDashboard.putNumber("right_rot", b.getRotation().getDegrees());

        double cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
        double sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());
        Rotation2d visionRot = new Rotation2d(Math.atan2(sinAvg, cosAvg));

        return new Pose2d(avgX, avgY, visionRot);
    }

    private Pose2d averagePoseRot(Pose2d avgPose, Pose2d a, Pose2d b) {
        double cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
        double sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());
        Rotation2d visionRot = new Rotation2d(Math.atan2(sinAvg, cosAvg));

        return new Pose2d(avgPose.getX(), avgPose.getY(), visionRot);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }

        visionPipeline();
    }

    @Override
    public void simulationPeriodic() {
        updateSimState(0.02, RobotController.getBatteryVoltage());
    }

    // -------------------------------------------------------------------------
    // Vision measurement overrides (timestamp correction)
    // -------------------------------------------------------------------------

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(
                visionRobotPoseMeters,
                Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs
        );
    }
}