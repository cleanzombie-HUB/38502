package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * ForwardVelocityTuner measures maximum forward velocity and acceleration.
 * <p>
 * Drives forward at full power for a set distance while tracking position with
 * 2-Wheel Odometry.
 */
@Autonomous(name = "Pedro Pathing: Forward Velocity Tuner", group = "Pedro Pathing Tuning")
public class ForwardVelocityTuner extends LinearOpMode {

    public static double DISTANCE = 48.0; // Target distance in inches

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        telemetry.addData("Status", "Ready. Press START to begin Forward Velocity Test.");
        telemetry.addData("Target Distance (inches)", DISTANCE);
        telemetry.update();

        waitForStart();

        localizer.reset();
        ElapsedTime timer = new ElapsedTime();
        double maxVelocity = 0.0;
        double startX = localizer.pose().x();

        while (opModeIsActive() && Math.abs(localizer.pose().x() - startX) < DISTANCE) {
            drivetrain.drive(new DrivePowers(1.0, 0.0, 0.0), false);
            localizer.update();

            double currentVelocity = localizer.state().velocity().linSpeed();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("Current X (inches)", "%.2f", localizer.pose().x());
            telemetry.addData("Current Velocity (in/s)", "%.2f", currentVelocity);
            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
            telemetry.update();
        }

        drivetrain.stop();
        double elapsedTime = timer.seconds();

        while (opModeIsActive()) {
            telemetry.addData("Test Complete", "Forward Velocity Test finished.");
            telemetry.addData("Final Distance (inches)", "%.2f", localizer.pose().x() - startX);
            telemetry.addData("Elapsed Time (s)", "%.2f", elapsedTime);
            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
            telemetry.update();
        }
    }
}
