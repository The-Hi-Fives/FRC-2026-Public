package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.GeometryUtil;
import frc.util.ManualDriveInput;

public class AimAndDriveCommand extends Command {
    private static final Angle kAimTolerance = Degrees.of(5);

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;

    private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
            .withRotationalDeadband(Driving.kPIDRotationDeadband)
            .withMaxAbsRotationalRate(Driving.kMaxRotationalRate.times(1.2))
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
            .withHeadingPID(4.5, 0, 0.25);

    public AimAndDriveCommand(
            Swerve swerve,
            DoubleSupplier forwardInput,
            DoubleSupplier leftInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput);
        addRequirements(swerve);
    }

    public AimAndDriveCommand(Swerve swerve) {
        this(swerve, () -> 0, () -> 0);
    }

    public boolean isAimed() {
        final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;
        final Rotation2d currentHeadingInBlueAlliancePerspective = swerve.getPose().getRotation();
        final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        // final Rotation2d currentHeadingInOperatorPerspective = currentHeadingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()).minus(swerve.getPose().getRotation());
        SmartDashboard.putNumber("targetHeading", targetHeading.getDegrees());
        SmartDashboard.putNumber("heading", targetHeading.getDegrees() - currentHeadingInOperatorPerspective.getDegrees());
        return GeometryUtil.isNear(targetHeading, currentHeadingInOperatorPerspective, kAimTolerance);
    }

    private Rotation2d getDirectionToHub() {
        final Translation2d robotPosition = swerve.getPose().getTranslation();
        @SuppressWarnings("unchecked")
        final HashMap<String, Translation2d> fieldCords = Landmarks.hubPosition();
        
        
        SmartDashboard.putNumber("robotPosition.x", robotPosition.getX());
        SmartDashboard.putNumber("robotPosition.y", robotPosition.getY());

        boolean inMiddle = false;
        if(robotPosition.getX() > Landmarks.inchesToMeters(182.11) && robotPosition.getX() < Landmarks.inchesToMeters(469.11))
        {
            inMiddle = true;
        }


        // final Translation2d hubPosition = Landmarks.hubPosition();
        final Translation2d hubPosition = fieldCords.get("HUB_POSE");
        if(inMiddle)
        {
            if(robotPosition.getY() < Landmarks.inchesToMeters(158.84))
            {
                final Rotation2d allianceDirectionInBluePerspective = fieldCords.get("PASS_RIGHT").minus(robotPosition).getAngle();
                final Rotation2d allianceDirectioninOperatorPerspective = allianceDirectionInBluePerspective.rotateBy(swerve.getOperatorForwardDirection());
                return allianceDirectioninOperatorPerspective;
            }
            else
            {
                final Rotation2d allianceDirectionInBluePerspective = fieldCords.get("PASS_LEFT").minus(robotPosition).getAngle();
                final Rotation2d allianceDirectioninOperatorPerspective = allianceDirectionInBluePerspective.rotateBy(swerve.getOperatorForwardDirection());
                return allianceDirectioninOperatorPerspective;
            }
        }

        // final Rotation2d targetHeading = fieldCentricFacingAngleRequest.TargetDirection;

        final Rotation2d hubDirectionInBlueAlliancePerspective = hubPosition.minus(robotPosition).getAngle();
        SmartDashboard.putNumber("hub direction", hubDirectionInBlueAlliancePerspective.getDegrees());
        SmartDashboard.putNumber("robot rotation", swerve.getPose().getRotation().getDegrees());
        // final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection()).minus(swerve.getPose().getRotation());
        final Rotation2d hubDirectionInOperatorPerspective = hubDirectionInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        SmartDashboard.putNumber("hubDirectionInOperatorPerspective", hubDirectionInOperatorPerspective.getDegrees());

        return hubDirectionInOperatorPerspective;
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        // Rotation2d directionToHub = getDirectionToHub();
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