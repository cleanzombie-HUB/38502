package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * LateralVelocityTuner measures maximum lateral (strafing) velocity and acceleration.
 * <p>
 * Strafes left at full power for a set distance while tracking position with 2-Wheel Odometry.
 */
@Autonomous(name = "Pedro Pathing: Lateral Velocity Tuner", group = "Pedro Pathing Tuning")
public class LateralVelocityTuner extends LinearOpMode {

    public static double DISTANCE = 48.0; // Target distance in inches

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        telemetry.addData("Status", "Ready. Press START to begin Lateral Velocity Test.");
        telemetry.addData("Target Distance (inches)", DISTANCE);
        telemetry.update();

        waitForStart();

        localizer.reset();
        ElapsedTime timer = new ElapsedTime();
        double maxVelocity = 0.0;
        double startY = localizer.pose().y();

        while (opModeIsActive() && Math.abs(localizer.pose().y() - startY) < DISTANCE) {
            drivetrain.drive(new DrivePowers(0.0, 1.0, 0.0), false);
            localizer.update();

            double currentVelocity = localizer.state().velocity().linSpeed();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("Current Y (inches)", "%.2f", localizer.pose().y());
            telemetry.addData("Current Velocity (in/s)", "%.2f", currentVelocity);
            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
            telemetry.update();
        }

        drivetrain.stop();
        double elapsedTime = timer.seconds();

        while (opModeIsActive()) {
            telemetry.addData("Test Complete", "Lateral Velocity Test finished.");
            telemetry.addData("Final Distance (inches)", "%.2f", localizer.pose().y() - startY);
            telemetry.addData("Elapsed Time (s)", "%.2f", elapsedTime);
            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
            telemetry.update();
        }
    }
}
