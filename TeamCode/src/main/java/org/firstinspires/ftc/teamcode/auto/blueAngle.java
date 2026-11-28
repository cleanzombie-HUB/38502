package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.helper.PathsPedro;
import org.firstinspires.ftc.teamcode.helper.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

/**
 * Autonomous OpMode for Blue Alliance testing Pedro Pathing (mirrored).
 */
@Autonomous(name = "Blue Angle (Path Test)", group = "Flower")
public class blueAngle extends LinearOpMode {

    private static double MAX_POWER = 1.0;
    private static double MIN_POWER = 0.4;

    private Follower follower;
    private ForesightConfig foresightConfig;
    private PathsPedro.BlueFlowerAngle pathsPedro;

    private int pathState = 0;
    private boolean pathStarted = false;

    public void setMaxPower(double maxPower) {
        if (foresightConfig != null) {
            foresightConfig.maxPathSpeed.set(maxPower);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    /**
     * Follows a path and advances to nextState ONLY when path execution is completely finished.
     */
    public void followPath(Path path, int nextState) {
        if (!pathStarted) {
            follower.follow(path);
            pathStarted = true;
        } else if (!follower.isBusy() && follower.atParametricEnd()) {
            pathStarted = false;
            setPathState(nextState);
        }
    }

    /**
     * Sequential path execution for path testing.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start 1: p1toflower (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p1toflower(), 1);
                break;

            case 1:
                // Start 2: p2flowerintake (MIN POWER = 0.4)
                setMaxPower(MIN_POWER);
                followPath(pathsPedro.p2flowerintake(), 2);
                break;

            case 2:
                // Start 3: p3toshoot (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p3toshoot(), 3);
                break;

            case 3:
                // Start 4: p4shoot (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p4shoot(), 4);
                break;

            case 4:
                // Start 5: p5tointake (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p5tointake(), 5);
                break;

            case 5:
                // Start 6: p6flowerintake (MIN POWER = 0.4)
                setMaxPower(MIN_POWER);
                followPath(pathsPedro.p6flowerintake(), 6);
                break;

            case 6:
                // Start 7: p7shoot (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p7shoot(), 7);
                break;

            case 7:
                // Start 8: p8park (MAX POWER)
                setMaxPower(MAX_POWER);
                followPath(pathsPedro.p8park(), 8);
                break;

            case 8:
                // Path sequence complete
                setPathState(-1);
                break;

            default:
                break;
        }
    }

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        foresightConfig = Constants.createForesightConfig(MAX_POWER);
        Foresight foresight = new Foresight(foresightConfig);
        follower = new Follower(localizer, drivetrain, foresight);
        follower.holdEnd.set(true);

        pathsPedro = new PathsPedro.BlueFlowerAngle();
        follower.setPose(pathsPedro.getStartPose());

        telemetry.addData("Status", "Blue Angle Path Test Initialized");
        telemetry.addData("Start Pose X", "%.2f", pathsPedro.getStartPose().x());
        telemetry.addData("Start Pose Y", "%.2f", pathsPedro.getStartPose().y());
        telemetry.addData("Start Heading (deg)", "%.2f", Math.toDegrees(pathsPedro.getStartPose().heading()));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        setPathState(0);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();

            Pose currentPose = follower.pose();
            telemetry.addData("Path State", pathState);
            telemetry.addData("Follower Busy", follower.isBusy());
            telemetry.addData("Pose X", "%.2f", currentPose.x());
            telemetry.addData("Pose Y", "%.2f", currentPose.y());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading()));
            telemetry.update();
        }

        if (follower != null) {
            RobotState.teleopStartingPose = follower.pose();
        }
    }
}
