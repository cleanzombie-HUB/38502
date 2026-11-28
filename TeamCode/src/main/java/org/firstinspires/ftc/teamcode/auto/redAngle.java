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
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helper.PathsPedro;
import org.firstinspires.ftc.teamcode.helper.RobotState;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

@Autonomous(name = "Red Angle (Path Test)", group = "Flower")
public class redAngle extends LinearOpMode {

    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.4;

    private Follower follower;
    private ForesightConfig foresightConfig;
    private PathsPedro.RedFlowerAngle pathsPedro;

    private Path p1, p2, p3, p4, p5, p6, p7, p8;

    private int pathState = 0;
    private boolean pathStarted = false;

    public void setMaxPower(double maxPower) {
        if (foresightConfig != null) {
            foresightConfig.maxPathSpeed.set(maxPower);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathStarted = false;
    }

    public void followPath(Path path, int nextState) {
        if (!pathStarted) {
            follower.follow(path);
            pathStarted = true;
        } else if (!follower.isBusy()) {
            setPathState(nextState);
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setMaxPower(MAX_POWER);
                followPath(p1, 1);
                break;
            case 1:
                setMaxPower(MIN_POWER);
                followPath(p2, 2);
                break;
            case 2:
                setMaxPower(MAX_POWER);
                followPath(p3, 3);
                break;
            case 3:
                setMaxPower(MAX_POWER);
                followPath(p4, 4);
                break;
            case 4:
                setMaxPower(MAX_POWER);
                followPath(p5, 5);
                break;
            case 5:
                setMaxPower(MIN_POWER);
                followPath(p6, 6);
                break;
            case 6:
                setMaxPower(MAX_POWER);
                followPath(p7, 7);
                break;
            case 7:
                setMaxPower(MAX_POWER);
                followPath(p8, 8);
                break;
            case 8:
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

        // Direct references to verify actual power sent to hardwareMap motors
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRight");

        foresightConfig = Constants.createForesightConfig(MAX_POWER);
        Foresight foresight = new Foresight(foresightConfig);
        follower = new Follower(localizer, drivetrain, foresight);
        follower.holdEnd.set(true);

        pathsPedro = new PathsPedro.RedFlowerAngle();

        p1 = pathsPedro.p1toflower();
        p2 = pathsPedro.p2flowerintake();
        p3 = pathsPedro.p3toshoot();
        p4 = pathsPedro.p4shoot();
        p5 = pathsPedro.p5tointake();
        p6 = pathsPedro.p6flowerintake();
        p7 = pathsPedro.p7shoot();
        p8 = pathsPedro.p8park();

        follower.setPose(pathsPedro.getStartPose());

        telemetry.addData("Status", "Red Angle Path Test Initialized");
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
            telemetry.addData("Pose X / Y", "%.2f / %.2f", currentPose.x(), currentPose.y());
            telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(currentPose.heading()));
            telemetry.addData("Power FL / FR", "%.2f / %.2f", fl.getPower(), fr.getPower());
            telemetry.addData("Power BL / BR", "%.2f / %.2f", bl.getPower(), br.getPower());
            telemetry.update();
        }

        if (follower != null) {
            RobotState.teleopStartingPose = follower.pose();
        }
    }
}