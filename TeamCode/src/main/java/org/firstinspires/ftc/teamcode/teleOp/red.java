package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.helper.subsystems;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class red extends LinearOpMode {

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

        // Initialize Subsystems for RED Alliance
        subsystems robotSubsystems = new subsystems(hardwareMap, subsystems.Alliance.RED);

        telemetry.addData("Status", "Red TeleOp Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Standard Direct Mecanum Drivetrain Control (Gamepad 1)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 2. Update Odometry & Automatically Target Closest Hive & Adjust Hood
            robotSubsystems.updateOdometry();
            robotSubsystems.updateHoodAutomatically();

            // 3. Dedicated Shooter & Servo Gate Controls
            if (gamepad2.right_bumper) {
                // Shoot Pollen: Start Pollen Flywheel, Open Servo Gate, & Run Intake to Feed
                robotSubsystems.startShooter1Pollen();
                robotSubsystems.openGate();
                robotSubsystems.intakeIn();
            } else if (gamepad2.y) {
                // Shoot Nectar: Start Nectar Flywheel, Open Servo Gate, & Run Intake to Feed
                robotSubsystems.startShooter2Nectar();
                robotSubsystems.openGate();
                robotSubsystems.intakeIn();
            } else if (gamepad2.left_bumper) {
                // Stop Shooters & Close Servo Gate
                robotSubsystems.stopShooters();
                robotSubsystems.stopIntake();
            } else {
                // Default Intake Controls when not shooting (Gate remains CLOSED while intaking)
                if (gamepad2.right_trigger > 0.1) {
                    robotSubsystems.closeGate();
                    robotSubsystems.setIntakePower(subsystems.INTAKE_POWER * gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.1) {
                    robotSubsystems.closeGate();
                    robotSubsystems.setIntakePower(-subsystems.INTAKE_POWER * gamepad2.left_trigger);
                } else {
                    robotSubsystems.stopIntake();
                }
            }

            // Telemetry reporting
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Auto-Selected Hive", robotSubsystems.getActiveHive());
            telemetry.addData("Distance to Hive (in)", "%.2f", robotSubsystems.getDistanceFromHive());
            telemetry.addData("Hood Position", "%.2f", robotSubsystems.getHoodCurrentPosition());
            telemetry.addData("Gate Position", "%.2f", robotSubsystems.getGateCurrentPosition());
            telemetry.addData("Pose X", "%.2f", robotSubsystems.getCurrentPose().x());
            telemetry.addData("Pose Y", "%.2f", robotSubsystems.getCurrentPose().y());
            telemetry.update();
        }
    }
}
