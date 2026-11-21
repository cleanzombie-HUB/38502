package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test OpMode (Drive + Intake + Sorting + Shooters 1 & 2)", group = "Testing")
public class testOp extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx intakeMotor;
    private DcMotorEx sortingMotor;
    private DcMotorEx shooter1Pollen;
    private DcMotorEx shooter2Nectar;

    // Studica Maverick Encoder Specs:
    // Shooter 1 (Pollen): 3.7:1 (1649 RPM) Motor Geared UP 2:1 to Flywheel (3298 Max RPM, 51.8 ticks per rev)
    public static final double SHOOTER1_TICKS_PER_REV = 51.8;

    // Shooter 2 (Nectar): 3.7:1 (1649 RPM) Motor Geared UP 2:1 to Flywheel (3298 Max RPM, 51.8 ticks per rev)
    public static final double SHOOTER2_TICKS_PER_REV = 51.8;

    // Dynamic Target RPM (Default 500 RPM, adjustable live using D-Pad)
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
        // Initialize Drivetrain Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortingMotor = hardwareMap.get(DcMotorEx.class, "sorting");
        sortingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1Pollen = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter1Pollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1Pollen.setDirection(DcMotorEx.Direction.REVERSE);

        shooter2Nectar = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooter2Nectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2Nectar.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addData("Status", "Test OpMode Initialized (Both Shooters: 3.7:1 Geared UP 2:1 -> 3298 Max RPM).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Standard Mecanum Drivetrain Control (Gamepad 1)
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1; // Reversed strafe
            double rx = -gamepad1.right_stick_x;     // Reversed turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 2. Dynamic Live RPM Adjustment via D-Pad (Up/Down = +/-500 RPM, Right/Left = +/-100 RPM)
            boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean currentDpadRight = gamepad1.dpad_right || gamepad2.dpad_right;
            boolean currentDpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;

            if (currentDpadUp && !lastDpadUp) {
                testRpm += 500.0;
            } else if (currentDpadDown && !lastDpadDown) {
                testRpm = Math.max(0.0, testRpm - 500.0);
            } else if (currentDpadRight && !lastDpadRight) {
                testRpm += 100.0;
            } else if (currentDpadLeft && !lastDpadLeft) {
                testRpm = Math.max(0.0, testRpm - 100.0);
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;
            lastDpadRight = currentDpadRight;
            lastDpadLeft = currentDpadLeft;

            // 3. Shooter Controls (Hold RB = Shooter 1 Pollen, Hold Y = Shooter 2 Nectar, Press LB = Stop Both)
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                shooter1Pollen.setVelocity(rpmToTicksPerSec(testRpm, SHOOTER1_TICKS_PER_REV));
                shooter2Nectar.setVelocity(0.0);
            } else if (gamepad1.y || gamepad2.y) {
                shooter2Nectar.setVelocity(rpmToTicksPerSec(testRpm, SHOOTER2_TICKS_PER_REV));
                shooter1Pollen.setVelocity(0.0);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                shooter1Pollen.setVelocity(0.0);
                shooter2Nectar.setVelocity(0.0);
            }

            // 4. Intake Control (Gamepad 1 or 2 Triggers)
            double intakePower = 0.0;
            if (gamepad1.right_trigger > 0.1) {
                intakePower = 1.0 * gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0.1) {
                intakePower = -1.0 * gamepad1.left_trigger;
            } else if (gamepad2.right_trigger > 0.1) {
                intakePower = 1.0 * gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0.1) {
                intakePower = -1.0 * gamepad2.left_trigger;
            }

            intakeMotor.setPower(intakePower);

            // 5. Sorting Motor Control (Gamepad 1 or 2 Hold A = IN, Hold B = OUT)
            double sortingPower = 0.0;
            if (gamepad1.a || gamepad2.a) {
                sortingPower = 1.0;
            } else if (gamepad1.b || gamepad2.b) {
                sortingPower = -1.0;
            }

            sortingMotor.setPower(sortingPower);

            // Live Telemetry Reporting
            telemetry.addData("Selected Target RPM", "%.0f RPM", testRpm);
            telemetry.addData("Shooter 1 (Geared UP 2:1 -> 3298 Max) Flywheel RPM", "%.0f RPM", ticksPerSecToRpm(shooter1Pollen.getVelocity(), SHOOTER1_TICKS_PER_REV));
            telemetry.addData("Shooter 2 (Geared UP 2:1 -> 3298 Max) Flywheel RPM", "%.0f RPM", ticksPerSecToRpm(shooter2Nectar.getVelocity(), SHOOTER2_TICKS_PER_REV));
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Sorting Power", "%.2f", sortingPower);
            telemetry.update();
        }
    }
}
