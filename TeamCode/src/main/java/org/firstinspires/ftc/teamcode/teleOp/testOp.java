package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Test OpMode (Drive + Intake + Sorting + Shooters 1 & 2)", group = "Testing")
public class testOp extends LinearOpMode {

    // Drivetrain Motors
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private IMU imu;

    // Mechanism Motors
    private DcMotorEx intakeMotor;
    private DcMotorEx sortingMotor;
    private DcMotorEx shooter1Pollen;
    private DcMotorEx shooter2Nectar;

    // Studica Maverick Encoder Specs:
    // Shooter 1 (Pollen): 1:1 6100 RPM Motor (112.0 4x quadrature ticks per rev)
    public static final double SHOOTER1_TICKS_PER_REV = 112.0;

    // Shooter 2 (Nectar): 1:1 6100 RPM Motor (112.0 4x quadrature ticks per rev)
    public static final double SHOOTER2_TICKS_PER_REV = 112.0;

    // Target Headings for Auto-Snap (Hive 1 = 0 Deg, Hive 2 = 180 Deg)
    private static final double HEADING_HIVE_1 = 0.0;
    private static final double HEADING_HIVE_2 = Math.PI;
    private static final double HEADING_P = 1.5;

    // Dynamic Target RPM (Default 500 RPM, adjustable live using Gamepad 2 D-Pad)
    private double testRpm = 500.0;

    // Button state tracking for debouncing D-Pad presses
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private double ticksPerSecToRpm(double ticksPerSec, double ticksPerRev) {
        return (ticksPerSec * 60.0) / ticksPerRev;
    }

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // 2. Initialize IMU (Control Hub Gyroscope) for Auto-Snap Heading Lock
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(imuParameters);

        // 3. Initialize Mechanism Motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortingMotor = hardwareMap.get(DcMotorEx.class, "sorting");
        sortingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1Pollen = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1Pollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1Pollen.setDirection(DcMotorEx.Direction.FORWARD);

        shooter2Nectar = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2Nectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2Nectar.setDirection(DcMotorEx.Direction.REVERSE);

        // Tuned Feedforward & Proportional Gain (F=13.5, P=12.0) to lock onto exact target RPM instantly
        PIDFCoefficients flywheelPidf = new PIDFCoefficients(12.0, 0.0, 0.0, 13.5);
        shooter1Pollen.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);
        shooter2Nectar.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);

        telemetry.addData("Status", "Test OpMode Initialized (Instant Speed Response Enabled).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // =========================================================================
            // GAMEPAD 1: DRIVER CONTROLS (Driving, Turning, & Auto-Snap Heading Lock)
            // =========================================================================
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1; // Reversed strafe
            double rx = -gamepad1.right_stick_x;     // Reversed turn

            // Auto-Snap Heading Lock (Hold Gamepad 1 X for Hive 1, Y for Hive 2 / Opposite)
            boolean snapHive1 = gamepad1.x;
            boolean snapHive2 = gamepad1.y;
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (snapHive1) {
                double headingError = AngleUnit.normalizeRadians(HEADING_HIVE_1 - currentHeading);
                rx = Math.max(-1.0, Math.min(1.0, headingError * HEADING_P)); // Snap to Hive 1 (0 Deg)
            } else if (snapHive2) {
                double headingError = AngleUnit.normalizeRadians(HEADING_HIVE_2 - currentHeading);
                rx = Math.max(-1.0, Math.min(1.0, headingError * HEADING_P)); // Snap to Hive 2 (180 Deg)
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // =========================================================================
            // GAMEPAD 2: OPERATOR CONTROLS (Intake, Sorting, Shooters, & Live RPM Tuning)
            // =========================================================================

            // 1. Live Target RPM Adjustment (Gamepad 2 D-Pad)
            boolean currentDpadUp = gamepad2.dpad_up;
            boolean currentDpadDown = gamepad2.dpad_down;
            boolean currentDpadRight = gamepad2.dpad_right;
            boolean currentDpadLeft = gamepad2.dpad_left;

            if (currentDpadUp && !lastDpadUp) {
                testRpm += 500.0; // D-Pad Up = +500 RPM
            } else if (currentDpadDown && !lastDpadDown) {
                testRpm = Math.max(0.0, testRpm - 500.0); // D-Pad Down = -500 RPM
            } else if (currentDpadRight && !lastDpadRight) {
                testRpm += 100.0; // D-Pad Right = +100 RPM
            } else if (currentDpadLeft && !lastDpadLeft) {
                testRpm = Math.max(0.0, testRpm - 100.0); // D-Pad Left = -100 RPM
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadRight = currentDpadRight;
            lastDpadLeft = currentDpadLeft;

            // 2. Shooter Test Controls (Instant Response via Feedforward F=13.5, P=12.0)
            if (gamepad2.right_bumper) {
                shooter1Pollen.setVelocity(rpmToTicksPerSec(testRpm, SHOOTER1_TICKS_PER_REV));
                shooter2Nectar.setVelocity(0.0);
            } else if (gamepad2.b && !gamepad2.start) {
                shooter2Nectar.setVelocity(rpmToTicksPerSec(testRpm, SHOOTER2_TICKS_PER_REV));
                shooter1Pollen.setVelocity(0.0);
            } else if (gamepad2.left_bumper) {
                shooter1Pollen.setVelocity(0.0);
                shooter2Nectar.setVelocity(0.0);
            }

            // 3. Floor Intake Control (Gamepad 2 Triggers)
            double intakePower = 0.0;
            if (gamepad2.right_trigger > 0.1) {
                intakePower = 1.0 * gamepad2.right_trigger; // Hold RT = Intake IN
            } else if (gamepad2.left_trigger > 0.1) {
                intakePower = -1.0 * gamepad2.left_trigger; // Hold LT = Intake OUT / Reverse
            }
            intakeMotor.setPower(intakePower);

            // 4. Sorting Motor Control (Gamepad 2 Hold A = IN)
            double sortingPower = 0.0;
            if (gamepad2.a) {
                sortingPower = 1.0; // Hold A = Sorting IN
            }
            sortingMotor.setPower(sortingPower);

            // =========================================================================
            // LIVE TELEMETRY REPORTING
            // =========================================================================
            telemetry.addData("Selected Target RPM", "%.0f RPM", testRpm);
            telemetry.addData("Shooter 1 (Pollen 1:1 6100) Actual RPM", "%.0f RPM", ticksPerSecToRpm(shooter1Pollen.getVelocity(), SHOOTER1_TICKS_PER_REV));
            telemetry.addData("Shooter 2 (Nectar 1:1 6100) Actual RPM", "%.0f RPM", ticksPerSecToRpm(shooter2Nectar.getVelocity(), SHOOTER2_TICKS_PER_REV));
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Sorting Power", "%.2f", sortingPower);
            telemetry.update();
        }
    }
}
