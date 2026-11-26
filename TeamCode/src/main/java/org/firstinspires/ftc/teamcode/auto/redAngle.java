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
 * Autonomous OpMode for Red Alliance using RedFlowerAngle Pedro Pathing.
 */
@Autonomous(name = "Red Angle", group = "Flower")
public class redAngle extends LinearOpMode {

    private static double MAX_POWER = 1.0;
    private static double MIN_POWER = 0.6;

    private Follower follower;
    private ForesightConfig foresightConfig;
    private PathsPedro.RedFlowerAngle pathsPedro;
    private rawSubsystem robotSubsystems;
    private ElapsedTime stateTimer;

    private int pathState = 0;

    public void setMaxPower(double maxPower) {
        MAX_POWER = maxPower;
        if (foresightConfig != null) {
            foresightConfig.maxPathSpeed.set(maxPower);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    /**
     * Autonomous state machine logic.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Shoot preloaded 4 pollen before moving
                robotSubsystems.shootPollenAuto();
                stateTimer.reset();
                setPathState(1);
                break;

            case 1:
                // Wait 2 seconds for preloaded pollen to shoot, then stop shooters and start p1toflower
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p1toflower());
                    setPathState(2);
                }
                break;

            case 2:
                // After p1toflower: put intake ramp down, start intake, and follow p2flowerintake
                if (!follower.isBusy()) {
                    robotSubsystems.lowerRamp();
                    robotSubsystems.intakeAuto();

                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.p2flowerintake());
                    setPathState(3);
                }
                break;

            case 3:
                // At end of p2flowerintake: reset timer to wait for intake
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    setPathState(4);
                }
                break;

            case 4:
                // Wait 2 seconds to fully intake, then stop intake and follow p3toshoot
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopIntakeAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p3toshoot());
                    setPathState(5);
                }
                break;

            case 5:
                // Follow p4shoot to driving position for shooting
                if (!follower.isBusy()) {
                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p4shoot());
                    setPathState(6);
                }
                break;

            case 6:
                // At end of p4shoot: shoot pollen
                if (!follower.isBusy()) {
                    robotSubsystems.shootPollenAuto();
                    stateTimer.reset();
                    setPathState(7);
                }
                break;

            case 7:
                // Wait 2 seconds for shooting, then put ramp down, stop shooters, and follow p5tointake
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();
                    robotSubsystems.lowerRamp();

                    setMaxPower(MIN_POWER);
                    follower.follow(pathsPedro.p5tointake());
                    setPathState(8);
                }
                break;

            case 8:
                // Start intake and follow p6flowerintake
                if (!follower.isBusy()) {
                    robotSubsystems.intakeAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p6flowerintake());
                    setPathState(9);
                }
                break;

            case 9:
                // At end of p6flowerintake: reset timer to wait for intake
                if (!follower.isBusy()) {
                    stateTimer.reset();
                    setPathState(10);
                }
                break;

            case 10:
                // Wait 2 seconds to fully intake, then stop intake and follow p7shoot
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopIntakeAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p7shoot());
                    setPathState(11);
                }
                break;

            case 11:
                // At end of p7shoot: shoot pollen again
                if (!follower.isBusy()) {
                    robotSubsystems.shootPollenAuto();
                    stateTimer.reset();
                    setPathState(12);
                }
                break;

            case 12:
                // Wait 2 seconds while shooting, then stop shooters and follow p8park
                if (stateTimer.seconds() >= 2.0) {
                    robotSubsystems.stopShooterAuto();

                    setMaxPower(MAX_POWER);
                    follower.follow(pathsPedro.p8park());
                    setPathState(13);
                }
                break;

            case 13:
                // End of p8park: complete sequence
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
    }

    @Override
    public void runOpMode() {
        TwoWheelLocalizer localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        Mecanum drivetrain = new Mecanum(hardwareMap, Constants.createMecanumConfig());

        foresightConfig = new ForesightConfig(c -> {
            c.maxPathSpeed.set(MAX_POWER);
        });
        Foresight foresight = new Foresight(foresightConfig);
        follower = new Follower(localizer, drivetrain, foresight);

        robotSubsystems = new rawSubsystem(hardwareMap);
        pathsPedro = new PathsPedro.RedFlowerAngle();
        stateTimer = new ElapsedTime();

        follower.setPose(pathsPedro.getStartPose());

        telemetry.addData("Status", "Red Angle Auto Initialized");
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
