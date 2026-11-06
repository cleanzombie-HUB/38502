package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp
public class testOp extends LinearOpMode {

    private DcMotorEx intakeMotor;

    @Override
    public void runOpMode() {
        // Initialize Mecanum Drivetrain
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        // Initialize Intake Motor ("intake")
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Test OpMode Initialized (Drive + Intake Only).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Mecanum Drivetrain Control (Gamepad 1)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Multiplier counteracts strafing friction
            double rx = gamepad1.right_stick_x;

            drivetrain.drive(new DrivePowers(y, x, rx), false);

            // 2. Intake Control (Gamepad 1 or Gamepad 2 Triggers)
            // Hold RT to Intake IN at full power (1.0), Hold LT to Intake OUT / Reverse at full power (-1.0)
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
