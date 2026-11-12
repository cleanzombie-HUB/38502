package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class testOp extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotorEx intakeMotor;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors directly (standard FTC hardwareMap)
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Intake Motor ("intake")
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Test OpMode Initialized (Drive + Intake Only).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Direct Mecanum Drivetrain Control (Gamepad 1)
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 2. Intake Control (Gamepad 1 or Gamepad 2 Triggers)
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

            // Telemetry reporting for testing
            telemetry.addData("Drive Y", "%.2f", y);
            telemetry.addData("Drive X", "%.2f", x);
            telemetry.addData("Turn RX", "%.2f", rx);
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.update();
        }
    }
}
