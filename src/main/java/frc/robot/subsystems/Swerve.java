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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {

    private Field2d m_field = new Field2d();
    private double gyroOffset = 0.0;
    private double lastRawYaw = 0.0;
    private boolean gyroCalibrated = false;
    private boolean poseInitialized = false;
    private int stableFrameCount = 0;
    private static final int INIT_THRESHOLD = 15; // ~0.5s of stable frames
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
    private final PIDController pathThetaController = new PIDController(5.5, 0, 0.35);

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

        SmartDashboard.putData("Field", m_field);
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

    public void visionPipeline() {
        Rotation2d rawGyroRotation = getRawGyroRotation();
        double yawVelRadPerSec = getYawVelocityRadPerSec();

        LimelightHelpers.PoseEstimate mt1_left  = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
        LimelightHelpers.PoseEstimate mt1_right = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");
        // ---- Cold Start Gate ----
        // Don't accept any measurements until gyro is calibrated from MT1
        if (!gyroCalibrated) {
            tryInitializeFromMT1(mt1_left, mt1_right, yawVelRadPerSec);
            return; // reject everything until gyro is trustworthy
        }

        if (!poseInitialized) {
            tryInitializePose(mt1_left, mt1_right, yawVelRadPerSec);
            return; // reject everything until pose is seeded
        }

        double headingDeg = getFieldRelativeHeadingDeg();
        // Update Limelight orientation
        LimelightHelpers.SetRobotOrientation("limelight-left",  headingDeg, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);

        // Get measurements
        LimelightHelpers.PoseEstimate mt2_left  = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        LimelightHelpers.PoseEstimate mt2_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

        // ----------------- Stage 1: Hard Reject -----------------
        boolean reject = false;

        if (mt2_left == null || mt2_right == null) return;
        if (mt1_left == null || mt1_right == null) return;
        if (mt2_left.tagCount < 1 && mt2_right.tagCount < 1) reject = true;

        // Reject if spinning too fast
        // if (Math.abs(yawVelRadPerSec) > Math.toRadians(120)) reject = true;

        if (!reject) {
            // ----------------- Combine Measurements -----------------
            Pose2d avgPose = averagePoseXY(mt2_left.pose, mt2_right.pose);
            // avgPose = averagePoseRot(avgPose, mt2_left.pose, mt2_right.pose);

            int tagCount    = Math.max(mt2_left.tagCount, mt2_right.tagCount);
            double distance = Math.min(mt2_left.avgTagDist, mt2_right.avgTagDist);

            // Hard reject tags beyond 4 meters during normal operation
            if (distance > 4.0) return;

            // Beyond 2.5m, only accept if multiple tags and high score
            if (distance > 2.0 && tagCount <= 2) return;

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

            if (score > 0.8) {
                accept = true;
            } else if (score > 0.5 && Math.abs(angleDiff) < 45) {
                accept = true;
            }

            // ----------------- Stage 6: Apply -----------------
            if (accept) {
                super.addVisionMeasurement(
                        avgPose,
                        Utils.fpgaToCurrentTime(mt1_right.timestampSeconds)
                );
            }

            // ----------------- Stage 7: Recovery Mode -----------------
            boolean multiTagStable =
                    tagCount >= 3 &&
                            Math.abs(angleDiff) < 5 &&
                            Math.abs(yawVelRadPerSec) < Math.toRadians(30);

            SmartDashboard.putBoolean("Updating from multitags", multiTagStable);
            // if (multiTagStable && !updateFromCam) {
            if(multiTagStable)
            {
                super.addVisionMeasurement(
                    avgPose,
                    Utils.fpgaToCurrentTime(mt2_right.timestampSeconds),
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2))
                );
                // resetPose(avgPose);
                // updateFromCam = true;
            }

            // ----------------- Debug -----------------
        //     SmartDashboard.putNumber("VisionScore",           score);
        //     SmartDashboard.putNumber("VisionAngleDiff",       angleDiff);
        //     SmartDashboard.putNumber("VisionXYStdDev",        xyStdDev);
        //     SmartDashboard.putNumber("VisionThetaStdDevDeg",  Math.toDegrees(thetaStdDev));
        //     SmartDashboard.putNumber("mt2_left.avgTagDist",   mt2_left.avgTagDist);
        //     SmartDashboard.putNumber("mt2_right.avgTagDist",  mt2_right.avgTagDist);
        //     SmartDashboard.putNumber("pose x",     getPose().getX());
        //     SmartDashboard.putNumber("pose y",     getPose().getY());
        //     SmartDashboard.putNumber("pose rot",   getPose().getRotation().getDegrees());
        //     SmartDashboard.putNumber("vision x",   avgPose.getX());
        //     SmartDashboard.putNumber("vision y",   avgPose.getY());
        //     SmartDashboard.putNumber("avgPose rot", avgPose.getRotation().getDegrees());
        //     SmartDashboard.putNumber("diff angle", angleDiff);
         }

        // SmartDashboard.putNumber("estimated x",    getPose().getX());
        // SmartDashboard.putNumber("estimated y",    getPose().getY());
        // SmartDashboard.putNumber("rawGyro",        rawGyroRotation.getDegrees());
        // SmartDashboard.putNumber("odometry rotation", getPose().getRotation().getDegrees());
    }

    private void tryInitializeFromMT1(
            LimelightHelpers.PoseEstimate mt1Left,
            LimelightHelpers.PoseEstimate mt1Right,
            double yawVelRadPerSec
    ) {

        // Need close tags, multiple tags, and robot nearly still
        boolean goodMT1 = mt1Left != null && mt1Right != null
                && mt1Left.tagCount >= 1 && mt1Right.tagCount >= 1
                && mt1Left.avgTagDist < 2.0 && mt1Right.avgTagDist < 2.0;
        boolean nearlyStationary = Math.abs(yawVelRadPerSec) < Math.toRadians(5);

        if (goodMT1 && nearlyStationary) {
            stableFrameCount++;
            if (stableFrameCount >= INIT_THRESHOLD) {
                Rotation2d mt1Heading = averagePoseRot(mt1Left.pose, mt1Right.pose);
                gyroOffset = mt1Heading.getDegrees()
                        - getPigeon2().getYaw().getValueAsDouble();
                gyroCalibrated = true;
                stableFrameCount = 0;
            }
        } else {
            stableFrameCount = 0;
        }

        SmartDashboard.putBoolean("GyroCalibrated", gyroCalibrated);
        SmartDashboard.putNumber("InitStableFrames", stableFrameCount);
    }

    private void tryInitializePose(
            LimelightHelpers.PoseEstimate mt1Left,
            LimelightHelpers.PoseEstimate mt1Right,
            double yawVelRadPerSec
    ) {
        boolean goodMT1 = mt1Left != null && mt1Right != null
                && mt1Left.tagCount >= 1 && mt1Right.tagCount >= 1
                && mt1Left.avgTagDist < 2.5 && mt1Right.avgTagDist < 2.5;
        boolean nearlyStationary = Math.abs(yawVelRadPerSec) < Math.toRadians(5);

        if (goodMT1 && nearlyStationary) {
            stableFrameCount++;
            if (stableFrameCount >= INIT_THRESHOLD) {
                Pose2d initPose = averagePoseXY(mt1Left.pose, mt1Right.pose);
                resetPose(new Pose2d(initPose.getTranslation(),
                        Rotation2d.fromDegrees(getFieldRelativeHeadingDeg())));
                poseInitialized = true;
                stableFrameCount = 0;
            }
        } else {
            stableFrameCount = 0;
        }

        SmartDashboard.putBoolean("PoseInitialized", poseInitialized);
    }

    private double getFieldRelativeHeadingDeg() {
        return getPigeon2().getYaw().getValueAsDouble() + gyroOffset;
    }

    // -------------------------------------------------------------------------
    // Vision pose averaging helpers (migrated from Drive.java)
    // -------------------------------------------------------------------------

    private Pose2d averagePoseXY(Pose2d a, Pose2d b) {
        double avgX, avgY;
        double cosAvg, sinAvg;
        if(a.getX() == 0 || b.getX() == 0)
        {
            avgX = Math.max(a.getX(), b.getX());
            avgY = Math.max(a.getY(), b.getY());
        }
        else
        {
            avgX = (a.getX() + b.getX()) / 2.0;
            avgY = (a.getY() + b.getY()) / 2.0;
        }

        SmartDashboard.putNumber("left_rot",  a.getRotation().getDegrees());
        SmartDashboard.putNumber("right_rot", b.getRotation().getDegrees());


        if(a.getX() == 0)
        {
            cosAvg = Math.cos(b.getRotation().getRadians());
            sinAvg = Math.sin(b.getRotation().getRadians());
        }
        else if(b.getX() == 0)
        {
            cosAvg = Math.cos(a.getRotation().getRadians());
            sinAvg = Math.sin(a.getRotation().getRadians());
        }
        else
        {
            cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
            sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());
        }

        Rotation2d visionRot = new Rotation2d(Math.atan2(sinAvg, cosAvg));

        return new Pose2d(avgX, avgY, visionRot);
    }

    private Rotation2d averagePoseRot(Pose2d a, Pose2d b) {
        // double cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
        // double sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());
        double cosAvg, sinAvg;

        if(a.getX() == 0)
        {
            cosAvg = Math.cos(b.getRotation().getRadians());
            sinAvg = Math.sin(b.getRotation().getRadians());
        }
        else if(b.getX() == 0)
        {
            cosAvg = Math.cos(a.getRotation().getRadians());
            sinAvg = Math.sin(a.getRotation().getRadians());
        }
        else
        {
            cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
            sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());
        }
        return new Rotation2d(Math.atan2(sinAvg, cosAvg));
    }

    public void initializeForAuto(Pose2d startPose) {
        gyroOffset = startPose.getRotation().getDegrees()
                - getPigeon2().getYaw().getValueAsDouble();
        gyroCalibrated = true;
        poseInitialized = true;
        resetPose(startPose);
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

        double currentRawYaw = getPigeon2().getYaw().getValueAsDouble();
        double yawJump = Math.abs(currentRawYaw - lastRawYaw);
        if (yawJump > 90) { // sudden large jump indicates a reset
            gyroCalibrated = false; // force recalibration from vision
        }
        lastRawYaw = currentRawYaw;

        visionPipeline();

        m_field.setRobotPose(getPose());
        SmartDashboard.putData("Field", m_field);
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