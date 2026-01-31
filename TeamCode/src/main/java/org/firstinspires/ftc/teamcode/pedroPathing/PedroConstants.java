package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@IgnoreConfigurable
public class PedroConstants {

    // ---------------- FOLLOWER ----------------
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(29.364996186637175)
            .lateralZeroPowerAcceleration(6.53288508904374)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.006,0,0.00155,0.06,0.001))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0235,0,0.001089,0.6,0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(0.75,0,0.032,0.017))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.59,0,0.05,0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.46,0,0.0095,0.008))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(1,0,0.032,0.012))
            .mass(16.2);

    // ---------------- DRIVE (FIXED) ----------------
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(71.0745642954909)
            .yVelocity(57.26737002875861)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);



    // ---------------- CORRECT CONSTRAINTS FOR YOUR API ----------------
    public static PathConstraints pathConstraints =
            new PathConstraints(
                    0.8,    // tValue
                    40,     // velocity
                    0.12,   // translational
                    0.9,    // heading
                    2.0,    // timeout
                    9.7,    // braking
                    200,    // search limit
                    0.8     // braking start
            );

    // ---------------- PINPOINT ----------------
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5.5)
            .strafePodX(5.25)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // ---------------- BUILDER ----------------
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
