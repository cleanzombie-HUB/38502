package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    // =========================================================================
    // 1. LOCALIZER CONFIGURATION (2-Wheel Odometry + Control Hub IMU)
    // =========================================================================

    // REV Through Bore Encoder specification (8192 ticks per revolution)
    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_DIAMETER_INCHES = 38.0 / 25.4; // 38mm deadwheels (1.49606 inches)
    public static double TICKS_TO_INCHES = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    // 2-Wheel Odometry Offsets (in inches relative to robot center of rotation)
    // Parallel (forward) pod offset: positive = forward of center
    public static double X_POD_OFFSET = 1.0;
    // Perpendicular (strafe) pod offset: positive = left of center
    public static double Y_POD_OFFSET = 0.0;

    // =========================================================================
    // 2. TUNED VELOCITY & ACCELERATION LIMITS (Update after running Tuners)
    // =========================================================================
    public static double MAX_FORWARD_VELOCITY = 45.97; // in/s (measured from ForwardVelocityTuner)
    public static double MAX_LATERAL_VELOCITY = 33.26; // in/s (measured from LateralVelocityTuner)
    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(106.71); // rad/s (measured from TurnTest)

    // =========================================================================
    // 3. PID GAINS & FEEDFORWARDS (Update after tuning path follower)
    // =========================================================================
    // Translational PID
    public static double TRANSLATIONAL_P = 1;
    public static double TRANSLATIONAL_I = 0.1;
    public static double TRANSLATIONAL_D = 0.2;

    // Heading PID
    public static double HEADING_P = 2.0;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.1;

    public static TwoWheelConfig createTwoWheelConfig() {
        return new TwoWheelConfig(c -> {
            c.xPodName.set("intake"); // Hardware map port for parallel pod encoder (shared with sorting motor)
            c.yPodName.set("sorting"); // Hardware map port for perpendicular pod encoder (shared with intake motor)
            c.imuName.set("imu");

            c.xPodOffset.set(X_POD_OFFSET);
            c.yPodOffset.set(Y_POD_OFFSET);

            c.forwardTicksToInches.set(TICKS_TO_INCHES);
            c.strafeTicksToInches.set(TICKS_TO_INCHES);

            c.xPodDirection.set(Encoder.REVERSE);
            c.yPodDirection.set(Encoder.REVERSE);

            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )));
        });
    }

    public static MecanumConfig createMecanumConfig() {
        return new MecanumConfig(c -> {
            c.frontLeftName.set("frontLeft");
            c.backLeftName.set("backLeft");
            c.frontRightName.set("frontRight");
            c.backRightName.set("backRight");

            c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
    }

    public static com.pedropathing.algorithm.ForesightConfig createForesightConfig() {
        return createForesightConfig(1.0);
    }

    public static com.pedropathing.algorithm.ForesightConfig createForesightConfig(double maxPower) {
        return new com.pedropathing.algorithm.ForesightConfig(c -> {
            c.headingFeedback.set(com.pedropathing.controllers.Controller.pid(HEADING_P, HEADING_I, HEADING_D));
            c.headingStaticFF.set(com.pedropathing.controllers.Controller.zero);
            c.forwardTranslational.set(com.pedropathing.controllers.Controller.pid(TRANSLATIONAL_P, TRANSLATIONAL_I, TRANSLATIONAL_D));
            c.strafeTranslational.set(com.pedropathing.controllers.Controller.pid(TRANSLATIONAL_P, TRANSLATIONAL_I, TRANSLATIONAL_D));

            c.maxPathSpeed.set(maxPower);
            c.maxVelocityConstraint.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_LATERAL_VELOCITY);

            c.maxAccelerationConstraint.set(60.0);
            c.maxDecelerationConstraint.set(60.0);

            c.brake.set(com.pedropathing.controllers.Controller.pid(1.0 / MAX_FORWARD_VELOCITY, 0.0, 0.0));
            c.coast.set(com.pedropathing.controllers.Controller.proportionalFeedforward(1.0 / MAX_FORWARD_VELOCITY));

            c.holdPointTranslationalScaling.set(1.0);
            c.holdPointHeadingScaling.set(1.0);
            c.maxBrakingPower.set(1.0);
            c.maxDecelerationScale.set(1.0);
            c.brakeAggression.set(1.0);
            c.coastDownToVelocity.set(0.0);

            c.headingDeviationTolerance.set(Math.toRadians(2.0));
            c.translationalDeviationTolerance.set(0.5);

            c.brakeAtEnd.set(true);
            c.pathSkip.set(false);
            c.headingDriveRatio.set(1.0);
            c.cosineScale.set(true);

            c.naturalForwardDeceleration.set(30.0);
            c.naturalStrafeDeceleration.set(30.0);

            c.linearBrakeCoefficients.set(com.pedropathing.math.Matrix.diag(0.5 / MAX_FORWARD_VELOCITY, 0.5 / MAX_LATERAL_VELOCITY));
            c.quadraticBrakeCoefficients.set(com.pedropathing.math.Matrix.zero(2));
            c.headingBrakeCoefficients.set(com.pedropathing.math.Vector2D.zero());
        });
    }
}