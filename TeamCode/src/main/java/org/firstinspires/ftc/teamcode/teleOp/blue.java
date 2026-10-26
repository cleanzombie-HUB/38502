package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helper.subsystems;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@TeleOp(name = "Blue TeleOp", group = "Competition")
public class blue extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize Drivetrain and Subsystems for BLUE Alliance
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());
        subsystems robotSubsystems = new subsystems(hardwareMap, subsystems.Alliance.BLUE);

        // Track selected game piece (Default: Pollen)
        subsystems.GamePiece selectedGamePiece = subsystems.GamePiece.POLLEN;

        telemetry.addData("Status", "Blue TeleOp Initialized. Waiting for start.");
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

            // 3. Game Piece Selection via Gamepad 2 D-pad Up/Down
            if (gamepad2.dpad_up) {
                selectedGamePiece = subsystems.GamePiece.POLLEN;
            } else if (gamepad2.dpad_down) {
                selectedGamePiece = subsystems.GamePiece.NECTAR;
            }

            // 4. Shooter Controls (RB to start, LB to stop)
            if (gamepad2.right_bumper) {
                if (selectedGamePiece == subsystems.GamePiece.POLLEN) {
                    robotSubsystems.startShooter1Pollen();
                } else {
                    robotSubsystems.startShooter2Nectar();
                }
            } else if (gamepad2.left_bumper) {
                robotSubsystems.stopShooters();
            }

            // 5. Intake Controls (RT to intake in, LT to intake out/reverse)
            if (gamepad2.right_trigger > 0.1) {
                robotSubsystems.setIntakePower(subsystems.INTAKE_POWER * gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0.1) {
                robotSubsystems.setIntakePower(-subsystems.INTAKE_POWER * gamepad2.left_trigger);
            } else {
                robotSubsystems.stopIntake();
            }

            // Telemetry reporting
            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Auto-Selected Hive 1 or 2", robotSubsystems.getActiveHive());
            telemetry.addData("Selected Game Piece", selectedGamePiece);
            telemetry.addData("Distance to Hive (in)", "%.2f", robotSubsystems.getDistanceFromHive());
            telemetry.addData("Hood Position", "%.2f", robotSubsystems.getHoodCurrentPosition());
            telemetry.addData("Pose X", "%.2f", robotSubsystems.getCurrentPose().x());
            telemetry.addData("Pose Y", "%.2f", robotSubsystems.getCurrentPose().y());
            telemetry.update();
        }
            }
}
