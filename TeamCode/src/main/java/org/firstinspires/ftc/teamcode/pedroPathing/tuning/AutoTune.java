package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * AutoTune procedures and dashboard host for Pedro Pathing 3.
 * <p>
 * Connect to http://192.168.43.1:10158/ from a browser on the Robot Controller Wi-Fi network
 * to execute these procedures interactively from the web dashboard.
 */
public class AutoTune {

    // =========================================================================
    // 1. FORWARD VELOCITY TUNER PROCEDURE
    // =========================================================================
    @Tuner(name = "Forward Velocity Tuner")
    public static Procedure forwardVelocityTuner() {
        return new Procedure(
                "Forward Velocity Tuner",
                "Drives forward at full power for a specified distance to measure maximum forward velocity and acceleration."
        ) {
            @Override
            public void run() throws InterruptedException {
                Inputs inputs = inputs("Forward Velocity Parameters", "Configure target distance and motor power scale.");
                Inputs.NumberField<Double> distInput = inputs.d("Target Distance (inches)").withDefault(24.0);
                Inputs.NumberField<Double> powerInput = inputs.d("Motor Power Scale (0.1 - 1.0)").withDefault(1.0);
                awaitInputs(inputs);

                double targetDist = distInput.get();
                double powerScale = powerInput.get();

                confirmation(
                        "Ready for Motion",
                        "Ensure the robot is placed on the field with at least " + targetDist + " inches of clear path in front."
                );

                Double maxVel = runOpMode(new TuningOpMode<Double>("Forward Motion Test", "Driving forward...", true) {
                    @Override
                    protected Double runTuningOpMode() throws InterruptedException {
                        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
                        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

                        waitForStart();
                        localizer.reset();
                        double startX = localizer.pose().x();
                        double maxVelocity = 0.0;

                        while (opModeIsActive() && Math.abs(localizer.pose().x() - startX) < targetDist) {
                            drivetrain.drive(new DrivePowers(powerScale, 0.0, 0.0), false);
                            localizer.update();

                            double currentVel = localizer.state().velocity().linSpeed();
                            if (currentVel > maxVelocity) {
                                maxVelocity = currentVel;
                            }

                            telemetry.addData("Status", "Running Forward Test");
                            telemetry.addData("Distance (in)", "%.2f / %.2f", Math.abs(localizer.pose().x() - startX), targetDist);
                            telemetry.addData("Current Velocity (in/s)", "%.2f", currentVel);
                            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
                            telemetry.update();
                        }
                        drivetrain.stop();
                        return maxVelocity;
                    }
                });

                result("Max Forward Velocity (in/s)", String.format("%.2f", maxVel));
                code(Language.JAVA, "public static double MAX_FORWARD_VELOCITY = " + String.format("%.2f", maxVel) + ";");
            }
        };
    }

    // =========================================================================
    // 2. LATERAL VELOCITY TUNER PROCEDURE
    // =========================================================================
    @Tuner(name = "Lateral Velocity Tuner")
    public static Procedure lateralVelocityTuner() {
        return new Procedure(
                "Lateral Velocity Tuner",
                "Strafes left at full power for a specified distance to measure maximum lateral velocity."
        ) {
            @Override
            public void run() throws InterruptedException {
                Inputs inputs = inputs("Lateral Velocity Parameters", "Configure target strafe distance and motor power scale.");
                Inputs.NumberField<Double> distInput = inputs.d("Target Distance (inches)").withDefault(24.0);
                Inputs.NumberField<Double> powerInput = inputs.d("Motor Power Scale (0.1 - 1.0)").withDefault(1.0);
                awaitInputs(inputs);

                double targetDist = distInput.get();
                double powerScale = powerInput.get();

                confirmation(
                        "Ready for Motion",
                        "Ensure the robot has at least " + targetDist + " inches of clear path to its left."
                );

                Double maxVel = runOpMode(new TuningOpMode<Double>("Lateral Motion Test", "Strafing left...", true) {
                    @Override
                    protected Double runTuningOpMode() throws InterruptedException {
                        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
                        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

                        waitForStart();
                        localizer.reset();
                        double startY = localizer.pose().y();
                        double maxVelocity = 0.0;

                        while (opModeIsActive() && Math.abs(localizer.pose().y() - startY) < targetDist) {
                            drivetrain.drive(new DrivePowers(0.0, powerScale, 0.0), false);
                            localizer.update();

                            double currentVel = localizer.state().velocity().linSpeed();
                            if (currentVel > maxVelocity) {
                                maxVelocity = currentVel;
                            }

                            telemetry.addData("Status", "Running Lateral Test");
                            telemetry.addData("Distance (in)", "%.2f / %.2f", Math.abs(localizer.pose().y() - startY), targetDist);
                            telemetry.addData("Current Velocity (in/s)", "%.2f", currentVel);
                            telemetry.addData("Max Velocity (in/s)", "%.2f", maxVelocity);
                            telemetry.update();
                        }
                        drivetrain.stop();
                        return maxVelocity;
                    }
                });

                result("Max Lateral Velocity (in/s)", String.format("%.2f", maxVel));
                code(Language.JAVA, "public static double MAX_LATERAL_VELOCITY = " + String.format("%.2f", maxVel) + ";");
            }
        };
    }

    // =========================================================================
    // 3. TURN & ANGULAR VELOCITY TUNER PROCEDURE
    // =========================================================================
    @Tuner(name = "Turn & Angular Velocity Tuner")
    public static Procedure turnTuner() {
        return new Procedure(
                "Turn & Angular Velocity Tuner",
                "Rotates the robot counter-clockwise by a target angle to measure maximum angular velocity and verify IMU heading tracking."
        ) {
            @Override
            public void run() throws InterruptedException {
                Inputs inputs = inputs("Turn Test Parameters", "Configure target turn angle and power scale.");
                Inputs.NumberField<Double> angleInput = inputs.d("Target Angle (degrees)").withDefault(90.0);
                Inputs.NumberField<Double> powerInput = inputs.d("Turn Power Scale (0.1 - 1.0)").withDefault(0.5);
                awaitInputs(inputs);

                double targetAngleDeg = angleInput.get();
                double powerScale = powerInput.get();

                confirmation(
                        "Ready for Motion",
                        "Ensure the robot has clear space around it to rotate " + targetAngleDeg + " degrees."
                );

                Double maxAngVel = runOpMode(new TuningOpMode<Double>("Turn Motion Test", "Rotating...", true) {
                    @Override
                    protected Double runTuningOpMode() throws InterruptedException {
                        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
                        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

                        waitForStart();
                        localizer.reset();
                        double targetRad = Math.toRadians(targetAngleDeg);
                        double maxAngVelocity = 0.0;

                        while (opModeIsActive() && Math.abs(localizer.pose().heading()) < targetRad) {
                            drivetrain.drive(new DrivePowers(0.0, 0.0, powerScale), false);
                            localizer.update();

                            double currentAngVel = Math.abs(localizer.state().velocity().omega);
                            if (currentAngVel > maxAngVelocity) {
                                maxAngVelocity = currentAngVel;
                            }

                            telemetry.addData("Status", "Running Turn Test");
                            telemetry.addData("Heading (deg)", "%.1f / %.1f", Math.toDegrees(Math.abs(localizer.pose().heading())), targetAngleDeg);
                            telemetry.addData("Max Angular Vel (rad/s)", "%.2f", maxAngVelocity);
                            telemetry.update();
                        }
                        drivetrain.stop();
                        return maxAngVelocity;
                    }
                });

                result("Max Angular Velocity (rad/s)", String.format("%.4f", maxAngVel));
                code(Language.JAVA, "public static double MAX_ANGULAR_VELOCITY = Math.toRadians(" + String.format("%.2f", Math.toDegrees(maxAngVel)) + "); // " + String.format("%.4f", maxAngVel) + " rad/s");
            }
        };
    }

    // =========================================================================
    // 4. LOCALIZATION VERIFICATION PROCEDURE
    // =========================================================================
    @Tuner(name = "Localization Test & Verification")
    public static Procedure localizationTestProcedure() {
        return new Procedure(
                "Localization Test & Verification",
                "Interactively drive the robot using Gamepad 1 to verify 2-Wheel Odometry (X, Y, Heading) position tracking."
        ) {
            @Override
            public void run() throws InterruptedException {
                confirmation(
                        "Start Localization Test",
                        "Use Gamepad 1 joysticks to drive the robot around the field while observing live X, Y, and Heading values."
                );

                runOpMode(new TuningOpMode<Void>("Interactive Driving", "Verifying deadwheel odometry...", true) {
                    @Override
                    protected Void runTuningOpMode() throws InterruptedException {
                        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
                        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

                        waitForStart();
                        while (opModeIsActive()) {
                            double y = -gamepad1.left_stick_y;
                            double x = -gamepad1.left_stick_x * 1.1;
                            double rx = -gamepad1.right_stick_x;

                            drivetrain.drive(new DrivePowers(y, x, rx), false);
                            localizer.update();

                            telemetry.addData("X (inches)", "%.2f", localizer.pose().x());
                            telemetry.addData("Y (inches)", "%.2f", localizer.pose().y());
                            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(localizer.pose().heading()));
                            telemetry.update();
                        }
                        return null;
                    }
                });

                result("Status", "Localization verification complete.");
            }
        };
    }

    // =========================================================================
    // 5. AUTOTUNE DASHBOARD HOST OPMODE
    // =========================================================================
    @TeleOp(name = "Pedro Pathing: AutoTune Server", group = "Pedro Pathing Tuning")
    public static class AutoTuneServerHost extends LinearOpMode {
        @Override
        public void runOpMode() {
            TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
            Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

            telemetry.addLine("=== PEDRO PATHING 3 AUTOTUNE SERVER ===");
            telemetry.addLine("1. Connect computer or phone to Robot Controller Wi-Fi network.");
            telemetry.addLine("2. Open web browser to:");
            telemetry.addLine("   -> http://192.168.43.1:10158/");
            telemetry.addLine("3. Select and execute any registered AutoTune procedure.");
            telemetry.addLine("\nStatus: Server Active on Port 10158 (WebSocket: 12649)");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                localizer.update();
                telemetry.addLine("=== PEDRO PATHING AUTOTUNE ACTIVE ===");
                telemetry.addData("Web Dashboard URL", "http://192.168.43.1:10158/");
                telemetry.addData("Pose X (inches)", "%.2f", localizer.pose().x());
                telemetry.addData("Pose Y (inches)", "%.2f", localizer.pose().y());
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(localizer.pose().heading()));
                telemetry.update();
            }
        }
    }
}
