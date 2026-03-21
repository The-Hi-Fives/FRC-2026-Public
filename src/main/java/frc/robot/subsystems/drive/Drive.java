// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private double simYawDeg = 0.0;
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
        // SmartDashboard.putNumber("rawGyroRotation", rawGyroRotation.getDegrees());
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    visionPipeline();

//    boolean doRejectUpdate = false;
//
//      // LimelightHelpers.SetRobotOrientation("limelight-left", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
//      LimelightHelpers.SetRobotOrientation("limelight-left", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);
//      LimelightHelpers.PoseEstimate mt2_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
//      LimelightHelpers.SetRobotOrientation("limelight-right", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);
//      LimelightHelpers.PoseEstimate mt2_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
//
//      // if our angular velocity is greater than 360 degrees per second, ignore vision updates
//     if(Math.abs(gyroInputs.yawVelocityRadPerSec) > Math.toRadians(90))
//     {
//         doRejectUpdate = true;
//     }
//      if((mt2_left != null && mt2_right != null) && (mt2_left.tagCount >= 2 && mt2_right.tagCount >= 2))
//      {
//        Pose2d avgPose = averagePose(mt2_left.pose, mt2_right.pose);
//        SmartDashboard.putNumber("avgpose.x", avgPose.getX());
//        SmartDashboard.putNumber("avgpose.y", avgPose.getY());
//        SmartDashboard.putNumber("avgpose.rot", avgPose.getRotation().getDegrees());
//
////        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,Units.degreesToRadians(10)));
//        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,Units.degreesToRadians(25)));
//
//        Rotation2d angleDiff = rawGyroRotation.minus(avgPose.getRotation());
//        if(Math.abs(angleDiff.getDegrees()) < 10)
//        {
//          poseEstimator.addVisionMeasurement(
//                  avgPose,
//                  mt2_right.timestampSeconds);
//        }
////          poseEstimator.addVisionMeasurement(
////                  avgPose,
////                  mt2_right.timestampSeconds);
//      }
//
//      SmartDashboard.putNumber("mt2_left", mt2_left.pose.getRotation().getDegrees());
//      SmartDashboard.putNumber("mt2_right", mt2_right.pose.getRotation().getDegrees());
//
     SmartDashboard.putNumber("estimated x", poseEstimator.getEstimatedPosition().getX());
     SmartDashboard.putNumber("estimated y", poseEstimator.getEstimatedPosition().getY());
     SmartDashboard.putNumber("rawGyro", rawGyroRotation.getDegrees());
     SmartDashboard.putNumber("odometry rotation", getRotation().getDegrees());

  }

  private void visionPipeline()
  {
    // ===================== VISION FILTER PIPELINE =====================

// Update Limelight orientation
    LimelightHelpers.SetRobotOrientation("limelight-left", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation("limelight-right", rawGyroRotation.getDegrees(), 0, 0, 0, 0, 0);

// Get measurements
    LimelightHelpers.PoseEstimate mt2_left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
    LimelightHelpers.PoseEstimate mt2_right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

    LimelightHelpers.PoseEstimate mt1_left = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    LimelightHelpers.PoseEstimate mt1_right = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");


// ----------------- Stage 1: Hard Reject -----------------
    boolean reject = false;

    if (mt2_left == null || mt2_right == null) return;
    if (mt2_left.tagCount < 1 && mt2_right.tagCount < 1) reject = true;

// Reject if spinning too fast
    if (Math.abs(gyroInputs.yawVelocityRadPerSec) > Math.toRadians(120)) reject = true;

    if (!reject) {

      // ----------------- Combine Measurements -----------------
      Pose2d avgPose = averagePoseXY(mt1_left.pose, mt1_right.pose);
      avgPose = averagePoseRot(avgPose, mt2_left.pose, mt2_right.pose);


      int tagCount = Math.max(mt2_left.tagCount, mt2_right.tagCount);
      double distance = Math.min(mt2_left.avgTagDist, mt2_right.avgTagDist);

      // ----------------- Stage 2: Compute Metrics -----------------
      double angleDiff =
              rawGyroRotation.minus(avgPose.getRotation()).getDegrees();

      // ----------------- Stage 3: Score -----------------
      double score = 1.0;

      // Tag count weight
      score *= Math.min(tagCount / 2.0, 1.0);

      // Distance weight (6m = low trust)
      score *= Math.max(0.0, 1.0 - (distance / 6.0));

      // Angle agreement weight
      score *= Math.max(0.0, 1.0 - (Math.abs(angleDiff) / 30.0));

      // ----------------- Stage 4: Convert to Std Devs -----------------
      double xyStdDev = 0.3 + (1.5 * (1.0 - score)); // meters
      double thetaStdDev = Units.degreesToRadians(10 + (60 * (1.0 - score))); // radians

      poseEstimator.setVisionMeasurementStdDevs(
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
      );

      // ----------------- Stage 5: Acceptance -----------------
      boolean accept = false;

      if (score > 0.7) {
        accept = true;
      } else if (score > 0.4 && Math.abs(angleDiff) < 45) {
        accept = true;
      }

      // ----------------- Stage 6: Apply -----------------
      if (accept) {
        poseEstimator.addVisionMeasurement(
                avgPose,
                mt2_right.timestampSeconds
        );
      }

      // ----------------- Stage 7: Recovery Mode -----------------
      boolean multiTagStable =
              tagCount >= 3 &&
                      Math.abs(angleDiff) < 5 &&
                      Math.abs(gyroInputs.yawVelocityRadPerSec) < Math.toRadians(30);

      SmartDashboard.putNumber("mt2_left.avgTagDist", mt2_left.avgTagDist);
      SmartDashboard.putNumber("mt2_right.avgTagDist", mt2_right.avgTagDist);
      SmartDashboard.putBoolean("Updating from multitags", multiTagStable);
      if (multiTagStable) {
        poseEstimator.resetPosition(
                rawGyroRotation,
                getModulePositions(),
                avgPose
        );
      }

      // ----------------- Debug -----------------
      SmartDashboard.putNumber("VisionScore", score);
      SmartDashboard.putNumber("VisionAngleDiff", angleDiff);
      SmartDashboard.putNumber("VisionXYStdDev", xyStdDev);
      SmartDashboard.putNumber("VisionThetaStdDevDeg", Math.toDegrees(thetaStdDev));
      SmartDashboard.putNumber("pose x", getPose().getX());
      SmartDashboard.putNumber("pose y", getPose().getY());
      SmartDashboard.putNumber("pose rot", getPose().getRotation().getDegrees());
      SmartDashboard.putNumber("vision x", avgPose.getX());
      SmartDashboard.putNumber("vision y", avgPose.getY());
      SmartDashboard.putNumber("avgPose rot", avgPose.getRotation().getDegrees());
      SmartDashboard.putNumber("diff angle", angleDiff);
    }
  }


  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
  
    private Pose2d averagePoseXY(Pose2d a, Pose2d b)
    {
      double avgX = (a.getX() + b.getX()) / 2.0;
      double avgY = (a.getY() + b.getY()) / 2.0;

      SmartDashboard.putNumber("left_rot", a.getRotation().getDegrees());
      SmartDashboard.putNumber("right_rot", b.getRotation().getDegrees());

      Rotation2d currentRotation = poseEstimator.getEstimatedPosition().getRotation();
      

      double cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
      double sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());

      Rotation2d visionRot = new Rotation2d(Math.atan2(sinAvg, cosAvg));
      Rotation2d avgRot = rawGyroRotation.interpolate(visionRot, .1);



      return new Pose2d(avgX, avgY, visionRot);
    }

    private Pose2d averagePoseRot(Pose2d avgPose, Pose2d a, Pose2d b)
    {
      double cosAvg = Math.cos(a.getRotation().getRadians()) + Math.cos(b.getRotation().getRadians());
      double sinAvg = Math.sin(a.getRotation().getRadians()) + Math.sin(b.getRotation().getRadians());

      Rotation2d visionRot = new Rotation2d(Math.atan2(sinAvg, cosAvg));
      Rotation2d avgRot = rawGyroRotation.interpolate(visionRot, .1);



      return new Pose2d(avgPose.getX(), avgPose.getY(), visionRot);
    }
}
