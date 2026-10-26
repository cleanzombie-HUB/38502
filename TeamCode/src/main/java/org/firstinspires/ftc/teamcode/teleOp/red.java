package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp(name = "Red TeleOp", group = "Competition")
public class red extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize Drivetrain and Subsystems for RED Alliance
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());
        subsystems robotSubsystems = new subsystems(hardwareMap, subsystems.Alliance.RED);

        telemetry.addData("Status", "Red TeleOp Initialized. Waiting for start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // 1. Drivetrain Control via gamepad1
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            drivetrain.drive(new DrivePowers(y, x, rx), false);

            // 2. Update Odometry & Automatically Target Closest Hive & Adjust Hood
            robotSubsystems.updateOdometry();
            robotSubsystems.updateHoodAutomatically();

            // 3. Dedicated Shooter Controls for holding both Pollen & Nectar
            if (gamepad2.right_bumper) {
                // Shoot Pollen (Biobuzz Shooter 1)
                robotSubsystems.startShooter1Pollen();
            } else if (gamepad2.y) {
                // Shoot Nectar (Shooter 2)
                robotSubsystems.startShooter2Nectar();
            } else if (gamepad2.left_bumper) {
                // Stop both shooters
                robotSubsystems.stopShooters();
            }

            // 4. Intake Controls (RT to intake in, LT to intake out/reverse)
            if (gamepad2.right_trigger > 0.1) {
                robotSubsystems.setIntakePower(subsystems.INTAKE_POWER * gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                robotSubsystems.setIntakePower(-subsystems.INTAKE_POWER * gamepad2.left_trigger);
            } else {
                robotSubsystems.stopIntake();
            }

            // Telemetry reporting
            telemetry.addData("Alliance", "RED");
            telemetry.addData("Auto-Selected Hive", robotSubsystems.getActiveHive());
            telemetry.addData("Distance to Hive (in)", "%.2f", robotSubsystems.getDistanceFromHive());
            telemetry.addData("Hood Position", "%.2f", robotSubsystems.getHoodCurrentPosition());
            telemetry.addData("Pose X", "%.2f", robotSubsystems.getCurrentPose().x());
            telemetry.addData("Pose Y", "%.2f", robotSubsystems.getCurrentPose().y());
            telemetry.update();
        }
    }
}
