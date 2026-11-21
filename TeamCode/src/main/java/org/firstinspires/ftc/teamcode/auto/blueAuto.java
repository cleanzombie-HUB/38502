package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helper.PathsPedro;
import org.firstinspires.ftc.teamcode.helper.RobotState;
import org.firstinspires.ftc.teamcode.helper.rawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * Autonomous OpMode for the Blue Alliance using Pedro Pathing.
 * Integrates shooters, intake, and ramp mechanisms via rawSubsystem.
 */
@Autonomous(name = "Blue Auto", group = "Flower")
public class blueAuto extends LinearOpMode {

    // Drive Power / Speed Constants (0.0 to 1.0)
    private static double MAX_POWER = 1.0;
    private static double MIN_POWER = 0.6;

    private Follower follower;
    private ForesightConfig foresightConfig;
    private PathsPedro pathsPedro;
    private rawSubsystem robotSubsystems;
    private ElapsedTime stateTimer;

    private int pathState = 0;

    /**
     * Dynamically caps the drivetrain output power/speed for path following.
     */
    public void setMaxPower(double maxPower) {
        MAX_POWER = maxPower;
        if (foresightConfig != null) {
            foresightConfig.maxPathSpeed.set(maxPower);
        }
    }

    /**
     * Executes the sequence of paths and mechanism controls for Blue Alliance.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // 1. As soon as auto starts, shoot preloaded 4 pollen before moving
                robotSubsystems.shootPollenAuto();
                stateTimer.reset();
                setPathState(1);
                break;

            case 1:
                // Wait 2.0s for initial 4 preloaded pollen to shoot, then stop shooters & start Path 1
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path1());
                    setPathState(2);
                }
                break;

            case 2:
                // 2. Right after Path 1, lower ramp and start intake at start of Path 2
                if (!follower.isBusy()) {
                    robotSubsystems.intakeAuto();
                    stateTimer.reset();
                    setPathState(3);
                }
                break;

            case 3:
                // Wait 1.5 seconds to intake before following Path 2
                if (stateTimer.seconds() >= 1.5) {
                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path2());
                    setPathState(4);
                }
                break;

            case 4:
                // 3. Path 3: Lift up ramp and stop intake
                if (!follower.isBusy()) {
                    robotSubsystems.stopIntakeAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path3());
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path4());
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path5());
                    setPathState(7);
                }
                break;

            case 7:
                // 4. Right at the end of Path 5: start shooters and shoot all 4 pollen
                if (!follower.isBusy()) {
                    robotSubsystems.shootPollenAuto();
                    stateTimer.reset();
                    setPathState(8);
                }
                break;

            case 8:
                // Wait 2.0s to shoot 4 pollen at end of Path 5, then stop shooters
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();

                    // 5. At Path 6: put down ramp and start intake for flower intake
                    robotSubsystems.intakeAuto();
                    stateTimer.reset();
                    setPathState(9);
                }
                break;

            case 9:
                // Wait 1.5 seconds to intake before following Path 6
                if (stateTimer.seconds() >= 1.5) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path6());
                    setPathState(10);
                }
                break;

            case 10:
                // Path 7: intake pollen from flower
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path7());
                    setPathState(11);
                }
                break;

            case 11:
                // 6. Path 8: stop intake but keep ramp down
                if (!follower.isBusy()) {
                    robotSubsystems.stopIntake();
                    robotSubsystems.lowerRamp();

                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path8());
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path9());
                    setPathState(13);
                }
                break;

            case 13:
                // At end of Path 9: shoot 4 pollen again
                if (!follower.isBusy()) {
                    robotSubsystems.shootPollenAuto();
                    stateTimer.reset();
                    setPathState(14);
                }
                break;

            case 14:
                // Wait 2.0s to shoot 4 pollen at end of Path 9, then stop shooters
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();

                    // Path 10: lift ramp and follow Path 10
                    robotSubsystems.raiseRamp();
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path10());
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    @Override
    public void runOpMode() {
        // Initialize Drivetrain, Localizer, and Pedro Pathing Follower
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        foresightConfig = new ForesightConfig(c -> {
            c.maxPathSpeed.set(MAX_POWER);
        });
        Foresight foresight = new Foresight(foresightConfig);
        follower = new Follower(localizer, drivetrain, foresight);

        // Initialize Raw Subsystem (No PedroPathing/Odometry dependencies)
        robotSubsystems = new rawSubsystem(hardwareMap);

        // Initialize Pedro Path definitions for Blue (mirrored)
        pathsPedro = new PathsPedro.Blue();

        // Initialize Timer
        stateTimer = new ElapsedTime();

        // Set starting pose from PathsPedro definition
        follower.setPose(pathsPedro.getStartPose());

        telemetry.addData("Status", "Blue Auto Initialized. Waiting for start.");
        telemetry.addData("Start Pose X", "%.2f", pathsPedro.getStartPose().x());
        telemetry.addData("Start Pose Y", "%.2f", pathsPedro.getStartPose().y());
        telemetry.addData("Start Heading (deg)", "%.2f", Math.toDegrees(pathsPedro.getStartPose().heading()));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        setPathState(0);

        while (opModeIsActive() && !isStopRequested()) {
            // Update follower state and drive outputs
            follower.update();

            // Run autonomous path sequence state machine
            autonomousPathUpdate();

            // Telemetry updates
            Pose currentPose = follower.pose();
            telemetry.addData("Path State", pathState);
            telemetry.addData("Current Drive Power Limit", "%.2f", MAX_POWER);
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Pose X", "%.2f", currentPose.x());
            telemetry.addData("Pose Y", "%.2f", currentPose.y());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }

        // Save final pose for TeleOp transition
        if (follower != null) {
            RobotState.teleopStartingPose = follower.pose();
        }
    }
}
