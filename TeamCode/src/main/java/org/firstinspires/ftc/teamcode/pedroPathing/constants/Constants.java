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
    public static double WHEEL_DIAMETER_INCHES = 1.37795; // 35mm deadwheels (adjust if 2.0" or 48mm)
    public static double TICKS_TO_INCHES = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    // 2-Wheel Odometry Offsets (in inches relative to robot center of rotation)
    // Parallel (forward) pod offset: positive = forward of center
    public static double X_POD_OFFSET = 0.0;
    // Perpendicular (strafe) pod offset: positive = left of center
    public static double Y_POD_OFFSET = 0.0;

    // =========================================================================
    // 2. TUNED VELOCITY & ACCELERATION LIMITS (Update after running Tuners)
    // =========================================================================
    public static double MAX_FORWARD_VELOCITY = 60.0; // in/s (measured from ForwardVelocityTuner)
    public static double MAX_LATERAL_VELOCITY = 45.0; // in/s (measured from LateralVelocityTuner)
    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(360.0); // rad/s (measured from TurnTest)

    // =========================================================================
    // 3. PID GAINS & FEEDFORWARDS (Update after tuning path follower)
    // =========================================================================
    // Translational PID
    public static double TRANSLATIONAL_P = 0.1;
    public static double TRANSLATIONAL_I = 0.0;
    public static double TRANSLATIONAL_D = 0.01;

    // Heading PID
    public static double HEADING_P = 2.0;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.1;

    public static TwoWheelConfig createTwoWheelConfig() {
        return new TwoWheelConfig(c -> {
            c.xPodName.set("leftFront"); // Hardware map port for parallel pod encoder
            c.yPodName.set("rightFront"); // Hardware map port for perpendicular pod encoder
            c.imuName.set("imu");

            c.xPodOffset.set(X_POD_OFFSET);
            c.yPodOffset.set(Y_POD_OFFSET);

            c.forwardTicksToInches.set(TICKS_TO_INCHES);
            c.strafeTicksToInches.set(TICKS_TO_INCHES);

            c.xPodDirection.set(Encoder.FORWARD);
            c.yPodDirection.set(Encoder.FORWARD);

            c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
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
}
