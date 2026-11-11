package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helper.PathsPedro;
import org.firstinspires.ftc.teamcode.helper.RobotState;
import org.firstinspires.ftc.teamcode.helper.rawSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * Autonomous OpMode for the Blue Alliance using Pedro Pathing.
 * Imports and executes the mirrored path sequence defined in PathsPedro.
 */
@Autonomous
public class blueAuto extends LinearOpMode {

    // Drive Power / Speed Constants (0.0 to 1.0)
    private static double MAX_POWER = 1.0;
    private static double MIN_POWER = 0.6;

    private Follower follower;
    private ForesightConfig foresightConfig;
    private PathsPedro pathsPedro;

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
     * Executes the sequence of paths defined in PathsPedro for the Blue alliance.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setMaxPower(MAX_POWER);
                follower.follow(pathsPedro.path1());
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path2());
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path3());
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path4());
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path5());
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path6());
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path7());
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.path8());
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path9());
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.path10());
                    setPathState(10);
                }
                break;
            case 10:
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

        // Initialize Raw Subsystem
        rawSubsystem robotSubsystems = new rawSubsystem(hardwareMap);

        // Initialize Pedro Path definitions for Blue (mirrored)
        pathsPedro = new PathsPedro.Blue();

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
