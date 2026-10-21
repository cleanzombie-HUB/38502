package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * LocalizationTest for 2-Wheel Odometry localizer tuning.
 * <p>
 * Use gamepad1 to drive the robot around while observing the robot's pose (X, Y, Heading)
 * on Driver Station telemetry.
 */
@TeleOp(name = "Pedro Pathing: Localization Test", group = "Pedro Pathing Tuning")
public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        telemetry.addData("Status", "Initialized 2-Wheel Deadwheel Localizer and Mecanum Drivetrain.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Drive using gamepad stick inputs
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            drivetrain.drive(new DrivePowers(y, x, rx), false);

            // Update 2-wheel deadwheel localizer
            localizer.update();
            Pose currentPose = localizer.pose();

            telemetry.addData("X (inches)", "%.2f", currentPose.x());
            telemetry.addData("Y (inches)", "%.2f", currentPose.y());
            telemetry.addData("Heading (rad)", "%.3f", currentPose.heading());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }
    }
}
