package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test OpMode (Drive + Intake + Sorting)", group = "Testing")
public class testOp extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx intakeMotor;
    private DcMotorEx sortingMotor;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors (Standard FTC Hardware Map)
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Intake Motor ("intake") & Sorting Motor ("sorting")
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sortingMotor = hardwareMap.get(DcMotorEx.class, "sorting");
        sortingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Test OpMode Initialized (Drive + Intake + Sorting).");
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

            // 2. Intake Control (Gamepad 1 or 2 Triggers)
            // Hold RT to Intake IN, Hold LT to Intake OUT / Reverse
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

            // 3. Hold-to-Run Sorting Motor Control (Hold A = Sorting IN, Hold B = Sorting OUT, Release = Stop)
            double sortingPower = 0.0;
            if (gamepad1.a || gamepad2.a) {
                sortingPower = 1.0;
            } else if (gamepad1.b || gamepad2.b) {
                sortingPower = -1.0;
            }

            sortingMotor.setPower(sortingPower);

            // Telemetry reporting for testing
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Sorting Power", "%.2f", sortingPower);
            telemetry.update();
        }
    }
}
