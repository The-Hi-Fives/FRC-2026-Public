package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.function.DoubleSupplier;
import java.util.stream.IntStream;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.Landmarks;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(1);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(Driving.kPIDRotationDeadband)
        .withMaxAbsRotationalRate(Driving.kMaxRotationalRate)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(5, 0, 0);

    public AimAndDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    private HashMap<String, Double> centerVals = new HashMap<String, Double>();

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    // private Rotation2d getDirectionToHub() {
    //     // // final Translation2d hubPosition = Landmarks.hubPosition();
    //     // final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
    //     // // final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
    //     // final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
    //     // return hubDirectionInOperatorPerspective;
    // }

    private Rotation2d getDirectionToHub()
    {
//        final Translation2d hubPosition = Landmarks.hubPosition();
        calcAngleToCenter();
        SmartDashboard.putNumber("angleToCenter", Math.toDegrees(centerVals.get("angleToCenter")));
        SmartDashboard.putNumber("avgDistToCenter", centerVals.get("avgDistToCenter"));
        SmartDashboard.putNumber("horizOffset", centerVals.get("horizOffset"));
        final Rotation2d centerPointAngle = new Rotation2d(-centerVals.get("angleToCenter"));
        // final Rotation2d centerPointAngle = new Rotation2d(-(2.00882*centerVals.get("angleToCenter") - 0.237045));
        // final Rotation2d centerPointAngle = new Rotation2d(-(-0.00134563*Math.pow(centerVals.get("angleToCenter"), 3)+0.00121509*Math.pow(centerVals.get("angleToCenter"), 2) + 2.10503*centerVals.get("angleToCenter")-0.313722));
        // final Translation2d centerHubPosition = new Translation2d(centerVals.get("avgDistToCenter"), centerPointAngle);
        // final Translation2d robotPosition = swerve.getState().Pose.getTranslation();
        // SmartDashboard.putNumber("robotposition", robotPosition.getAngle().getDegrees());
        // SmartDashboard.putNumber("hubdirection", centerHubPosition.minus(robotPosition).getAngle().getDegrees());
        // final Rotation2d hubDirectionInBlueAlliancePerspective = centerHubPosition.minus(robotPosition).getAngle();
        // SmartDashboard.putNumber("hubdirection swerve", hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()).getDegrees());
        // final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        // SmartDashboard.putNumber("rotate by", swerve.getOperatorForwardDirection().getDegrees());
        // final Rotation2d hubDirectionInOperatorPerspective = centerPointAngle.rotateBy(swerve.getOperatorForwardDirection());
        return centerPointAngle;
    }

    private void calcAngleToCenter()
    {
//        HashMap<String, Double> centerVals = calcOffsetAndDist();

//        double horizOffset = calcHorizOffset();
//        double avgDistToCenter = calcAvgDistToCenter();
        calcOffsetAndDist();

        // double angleToCenter = Math.asin(centerVals.get("horizOffset")/Math.sqrt(Math.pow(centerVals.get("horizOffset"), 2) + Math.pow(centerVals.get("avgDistToCenter"), 2)));
        double angleToCenter = Math.asin((centerVals.get("horizOffset"))/Math.sqrt(Math.pow(centerVals.get("horizOffset"), 2) + Math.pow(centerVals.get("avgDistToCenter"), 2)));
        SmartDashboard.putNumber("angleToCenter Calc", angleToCenter);
        centerVals.put("angleToCenter", angleToCenter);
//        return angleToCenter;
    }

    private void calcOffsetAndDist()
    {
        double distanceFromTag = 0;
        double angleTags = 0.0;

        LinkedList<Double> distanceToCenter = new LinkedList<Double>();
        LinkedList<Integer> detectedId = new LinkedList<Integer>();
        HashMap<Integer, Integer> tagMap = new HashMap<>();
        tagMap.put(25, 26);
        double avgDistToCenter = 0;
        // double horizOffset = 0;
        int[] blueValidTagsCenter = {18, 20, 21, 26};
        int[] blueValidTagsOffset = {19, 24, 25, 27};

        // LimelightHelpers.RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");
        // for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
        //     if(IntStream.of(blueValidTagsCenter).anyMatch(x -> x == detectedTag.id))
        //     {
        //         //Distance from tag in inches
        //         distanceFromTag = detectedTag.distToCamera * 39.37;
        //         SmartDashboard.putNumber("tag angle", detectedTag.txnc);
        //         distanceToCenter.add((distanceFromTag * Math.sin(Math.toRadians(90-detectedTag.txnc))) + 23.0315);
        //         // distanceToCenter.add(distanceFromTag + 23.0315);
        //         SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
        //         horizOffset = distanceFromTag * Math.sin(Math.toRadians(detectedTag.txnc));
        //     }
        //     // if(IntStream.of(blueValidTagsOffset).anyMatch(x -> x == detectedTag.id))
        //     // {
        //     //     distanceFromTag = detectedTag.distToCamera * 39.37;
        //     //     distanceToCenter.add(distanceFromTag + 26.9527);
        //     //     SmartDashboard.putNumber("Distance to Hub (inches)", distanceFromTag);
        //     //     horizOffset = distanceFromTag * Math.sin(Math.toRadians(detectedTag.txnc));
        //     // }
        // }
        // for(double dist : distanceToCenter)
        // {
        //     avgDistToCenter = avgDistToCenter + dist;
        // }
        // avgDistToCenter /= distanceToCenter.size();
        LimelightHelpers.RawFiducial[] detectedTags = LimelightHelpers.getRawFiducials("limelight");
        for (LimelightHelpers.RawFiducial detectedTag : detectedTags) {
            detectedId.add(detectedTag.id);
        }
        if(detectedId.contains(25) && detectedId.contains(26))
        {
            double id25Dist = 0.0;
            double id26Dist = 0.0;
            double angleBetweenTags = 0.0;
            for(LimelightHelpers.RawFiducial detectedTag : detectedTags)
            {
                if(detectedTag.id == 25)
                {
                    id25Dist = detectedTag.distToCamera*39.37;
                }
                else if (detectedTag.id == 26)
                {
                    id26Dist = detectedTag.distToCamera*39.37;
                }
            }
            angleBetweenTags = Math.acos((Math.pow(id25Dist, 2) + Math.pow(id26Dist, 2) - Math.pow(14, 2))/(2*id25Dist*id26Dist));
            double pointOutsideAngle = 90-Math.toDegrees(Math.asin((Math.sin(angleBetweenTags)*id25Dist)/id26Dist));
            double centerTagToHub = (id26Dist*Math.sin(Math.toRadians(90-pointOutsideAngle)));
            double centerTagToHubCenter = centerTagToHub + 23.0315;
            double horizOffset = Math.sqrt(Math.pow(centerTagToHub, 2) + Math.pow(id26Dist, 2));
            centerVals.put("avgDistToCenter", centerTagToHubCenter);
            centerVals.put("horizOffset", horizOffset);
        }


    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        swerve.setControl(
            fieldCentricFacingAngleRequest
                .withVelocityX(Driving.kMaxSpeed.times(input.forward))
                .withVelocityY(Driving.kMaxSpeed.times(input.left))
                .withTargetDirection(getDirectionToHub())
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
