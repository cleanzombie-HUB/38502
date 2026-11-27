package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.helper.rawSubsystem;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class blue extends LinearOpMode {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private IMU imu;

    // Target headings for Hive 1 vs Hive 2 (Opposite Side = 180 degrees / PI radians difference)
    private static final double HEADING_HIVE_1 = Math.PI;   // Facing Hive 1 (180 Deg)
    private static final double HEADING_HIVE_2 = 0.0;       // Facing Hive 2 / Opposite Side (0 Deg)
    private static final double HEADING_P = 1.5;            // Proportional gain for auto-snap turn

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

        // Initialize IMU (Control Hub Gyroscope) for Auto-Snap Heading Lock
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(imuParameters);

        // Initialize Raw Subsystem
        rawSubsystem robotSubsystems = new rawSubsystem(hardwareMap);

        telemetry.addData("Status", "Blue TeleOp Initialized.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Drivetrain Control (Gamepad 1)
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;

            // DUAL-HIVE AUTO-SNAP HEADING LOCK (Gamepad 1)
            // Hold X = Auto-Snap to Hive 1
            // Hold Y = Auto-Snap to Hive 2 / Opposite Side
            boolean snapHive1 = gamepad1.x;
            boolean snapHive2 = gamepad1.y;
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (snapHive1) {
                double headingError = AngleUnit.normalizeRadians(HEADING_HIVE_1 - currentHeading);
                rx = Math.max(-1.0, Math.min(1.0, headingError * HEADING_P)); // Snap to Hive 1
            } else if (snapHive2) {
                double headingError = AngleUnit.normalizeRadians(HEADING_HIVE_2 - currentHeading);
                rx = Math.max(-1.0, Math.min(1.0, headingError * HEADING_P)); // Snap to Hive 2 (Opposite side)
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 2. Preset Shooting Position Selection (Gamepad 2 D-Pad Up / Down)
            if (gamepad2.dpad_up) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.CLOSE);
            } else if (gamepad2.dpad_down) {
                robotSubsystems.setShootingPosition(rawSubsystem.ShootingPosition.FAR);
            }

            // 3. Flower Intake Ramp Control (Gamepad 2 D-Pad Right = Lower Ramp 200°, D-Pad Left = Raise Ramp)
            if (gamepad2.dpad_right) {
                robotSubsystems.lowerRamp(); // Lower ramp 200 degrees for flower intake
            } else if (gamepad2.dpad_left) {
                robotSubsystems.raiseRamp(); // Raise ramp stowed
            }

            // 4. Dedicated Shooter & Auto Sorting Controls (Gamepad 2)
            if (gamepad2.right_bumper) {
                // Shoot Pollen (Biobuzz Shooter 1)
                robotSubsystems.startShooter1Pollen();
                robotSubsystems.sortingIn();
            } else if (gamepad2.y) {
                // Shoot Nectar (Shooter 2)
                robotSubsystems.startShooter2Nectar();
                robotSubsystems.sortingIn();
            } else if (gamepad2.left_bumper) {
                // Stop Shooters & Sorting
                robotSubsystems.stopShooters();
                robotSubsystems.stopSorting();
            } else {
                // 5. Hold-to-Run Sorting Motor Controls (Hold A = Sorting IN, Hold B = Sorting OUT, Release = Stop)
                if (gamepad2.a) {
                    robotSubsystems.sortingIn();  // Hold A = Sorting IN
                } else if (gamepad2.b) {
                    robotSubsystems.sortingOut(); // Hold B = Sorting OUT / Reverse
                } else {
                    robotSubsystems.stopSorting(); // Release = Auto Stop
                }

                // 6. Floor Intake Controls (Gamepad 2 Triggers)
                if (gamepad2.right_trigger > 0.1) {
                    robotSubsystems.setIntakePower(rawSubsystem.INTAKE_POWER * gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.1) {
                    robotSubsystems.setIntakePower(-rawSubsystem.INTAKE_POWER * gamepad2.left_trigger);
                } else {
                    robotSubsystems.stopIntake();
                }
            }

            // Telemetry
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Auto-Snap Target", snapHive1 ? "HIVE 1 (180°)" : (snapHive2 ? "HIVE 2 / OPPOSITE (0°)" : "MANUAL DRIVING"));
            telemetry.addData("Heading (Deg)", "%.1f", Math.toDegrees(currentHeading));
            telemetry.addData("Shooting Preset", robotSubsystems.getShootingPosition());
            telemetry.addData("Ramp Position", "%.2f", robotSubsystems.getRampCurrentPosition());
            telemetry.update();
        }
    }
}
