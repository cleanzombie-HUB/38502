package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.rawSubsystem;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class red extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    @Override
    public void runOpMode() {
        // Initialize Drivetrain Motors (Standard FTC Hardware Map, no PedroPathing)
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize Raw Subsystem (No PedroPathing/Odometry dependencies)
        rawSubsystem robotSubsystems = new rawSubsystem(hardwareMap);

        telemetry.addData("Status", "Red TeleOp Initialized (Raw Subsystem & Standard Mecanum).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Standard Mecanum Drivetrain Control (Gamepad 1)
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

            // 2. Preset Shooting Position Selection (Gamepad 2 D-Pad)
            if (gamepad2.dpad_up) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.CLOSE);
            } else if (gamepad2.dpad_down) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.FAR);
            }

            // 3. Dedicated Shooter, Servo Gate, & Auto Sorting Controls (Gamepad 2)
            if (gamepad2.right_bumper) {
                // Shoot Pollen (Biobuzz Shooter 1)
                robotSubsystems.startShooter1Pollen();
                robotSubsystems.openGate();
                robotSubsystems.sortingIn();
            } else if (gamepad2.y) {
                // Shoot Nectar (Shooter 2)
                robotSubsystems.startShooter2Nectar();
                robotSubsystems.openGate();
                robotSubsystems.sortingIn();
            } else if (gamepad2.left_bumper) {
                // Stop Shooters, Sorting, & Close Servo Gate
                robotSubsystems.stopShooters();
                robotSubsystems.stopSorting();
            } else {
                // 4. Hold-to-Run Sorting Motor Controls (Hold A = Sorting IN, Hold B = Sorting OUT, Release = Stop)
                if (gamepad2.a) {
                    robotSubsystems.sortingIn();  // Hold A = Sorting IN
                } else if (gamepad2.b) {
                    robotSubsystems.sortingOut(); // Hold B = Sorting OUT / Reverse
                } else {
                    robotSubsystems.stopSorting(); // Release = Auto Stop
                }

                // 5. Floor Intake Controls (Gamepad 2 Triggers)
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

            // Telemetry
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Shooting Preset", robotSubsystems.getShootingPosition());
            telemetry.addData("Gate Position", "%.2f", robotSubsystems.getGateCurrentPosition());
            telemetry.update();
        }
    }
}
