package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.rawSubsystem;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class blue extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

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

        // Initialize Raw Subsystem
        rawSubsystem robotSubsystems = new rawSubsystem(hardwareMap);

        telemetry.addData("Status", "Blue TeleOp Initialized (rawSubsystem). Waiting for start.");
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

            // 2. Target Shooting Position Selection (D-Pad UP = CLOSE, D-Pad DOWN = FAR)
            if (gamepad2.dpad_up) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.CLOSE);
            } else if (gamepad2.dpad_down) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.FAR);
            }

            // 3. Dedicated Shooter & Servo Gate Controls (Gamepad 2)
            if (gamepad2.right_bumper) {
                // Shoot Pollen: Start Pollen Flywheel, Open Gate, & Run Intake to Feed
                robotSubsystems.startShooter1Pollen();
                robotSubsystems.openGate();
                robotSubsystems.intakeIn();
            } else if (gamepad2.y) {
                // Shoot Nectar: Start Nectar Flywheel, Open Gate, & Run Intake to Feed
                robotSubsystems.startShooter2Nectar();
                robotSubsystems.openGate();
                robotSubsystems.intakeIn();
            } else if (gamepad2.left_bumper) {
                // Stop Shooters & Intake, Close Gate
                robotSubsystems.stopShooters();
                robotSubsystems.stopIntake();
            } else {
                // Default Intake Controls when not shooting
                if (gamepad2.right_trigger > 0.1) {
                    robotSubsystems.closeGate();
                    robotSubsystems.setIntakePower(rawSubsystem.INTAKE_POWER * gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.1) {
                    robotSubsystems.closeGate();
                    robotSubsystems.setIntakePower(-rawSubsystem.INTAKE_POWER * gamepad2.left_trigger);
                } else {
                    robotSubsystems.stopIntake();
                }
            }

            // Telemetry reporting
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Shooting Position", robotSubsystems.getShootingPosition());
            telemetry.addData("Hood Position", "%.2f", robotSubsystems.getHoodCurrentPosition());
            telemetry.addData("Gate Position", "%.2f", robotSubsystems.getGateCurrentPosition());
            telemetry.update();
        }
    }
}
