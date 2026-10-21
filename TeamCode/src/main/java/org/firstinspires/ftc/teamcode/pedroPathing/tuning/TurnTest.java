package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * TurnTest verifies IMU heading tracking and angular tuning.
 * <p>
 * Rotates the robot counter-clockwise by a target angle (default 90 deg) and compares target vs actual.
 */
@Autonomous(name = "Pedro Pathing: Turn Test", group = "Pedro Pathing Tuning")
public class TurnTest extends LinearOpMode {

    public static double ANGLE_DEGREES = 90.0; // Target turn angle in degrees

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        telemetry.addData("Status", "Ready. Press START to begin Turn Test.");
        telemetry.addData("Target Angle (degrees)", ANGLE_DEGREES);
        telemetry.update();

        waitForStart();

        localizer.reset();
        double targetAngleRad = Math.toRadians(ANGLE_DEGREES);

        while (opModeIsActive() && Math.abs(localizer.pose().heading()) < targetAngleRad) {
            drivetrain.drive(new DrivePowers(0.0, 0.0, 0.5), false);
            localizer.update();

            telemetry.addData("Current Heading (deg)", "%.1f", Math.toDegrees(localizer.pose().heading()));
            telemetry.addData("Target Heading (deg)", "%.1f", ANGLE_DEGREES);
            telemetry.update();
        }

        drivetrain.stop();

        while (opModeIsActive()) {
            localizer.update();
            telemetry.addData("Test Complete", "Turn Test finished.");
            telemetry.addData("Final Heading (deg)", "%.2f", Math.toDegrees(localizer.pose().heading()));
            telemetry.addData("Error (deg)", "%.2f", Math.toDegrees(localizer.pose().heading()) - ANGLE_DEGREES);
            telemetry.update();
        }
    }
}
