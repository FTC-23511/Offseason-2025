package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "FL";
        FollowerConstants.leftRearMotorName = "BL";
        FollowerConstants.rightFrontMotorName = "FR";
        FollowerConstants.rightRearMotorName = "BR";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        // In Kg
        FollowerConstants.mass = 12.55;

        // All averages are 2 tests at 48", then 1 test at 60", then 1 test at 72"
        FollowerConstants.xMovement = (76.6523 + 77.8684 + 79.0379 + 82.1493) / 4;
        FollowerConstants.yMovement = (61.5339 + 60.3297 + 62.9171 + 63.3454) / 4;

        // All averages are 4 tests at 30 in/sec
        FollowerConstants.forwardZeroPowerAcceleration = (-38.6913 + -36.9557 + -35.9854 + -36.1437) / 4;
        FollowerConstants.lateralZeroPowerAcceleration = (-76.8218 + -64.3678 + -70.3518 + -71.4412) / 4;

        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.16,0,0.01,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.13,0,0.015,0); // @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(1.5,0,0.1,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(1.2,0,0.1,0); // @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.018,0,0.0000012,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.01,0,0.000001,0.6,0); // @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 3.5;
        FollowerConstants.centripetalScaling = 0.0004;

        // Controls pause at end of forward and backward test
        FollowerConstants.pathEndTimeoutConstraint = 50;
        FollowerConstants.pathEndTValueConstraint = 0.975;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
