package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {

    // REV Through Bore Encoder specification (8192 ticks per revolution)
    public static double TICKS_PER_REV = 8192.0;
    public static double WHEEL_DIAMETER_INCHES = 1.37795; // 35mm deadwheels (adjust if 2.0" or 48mm)
    public static double TICKS_TO_INCHES = (Math.PI * WHEEL_DIAMETER_INCHES) / TICKS_PER_REV;

    // 2-Wheel Odometry Offsets (in inches relative to robot center of rotation)
    // Forward (parallel) pod offset: positive = forward of center
    public static double X_POD_OFFSET = 0.0;
    // Strafe (perpendicular) pod offset: positive = left of center
    public static double Y_POD_OFFSET = 0.0;

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

            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        });
    }
}
