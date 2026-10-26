package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

public class subsystems {

    public enum Alliance {
        RED,
        BLUE
    }

    public enum GamePiece {
        POLLEN,
        NECTAR
    }

    public enum HiveTarget {
        HIVE_1,
        HIVE_2
    }

    // Hardware components
    private final DcMotorEx shooter1Pollen; // Shooter 1: Pollen (Biobuzz)
    private final DcMotorEx shooter2Nectar;  // Shooter 2: Nectar
    private final DcMotorEx intakeMotor;       // Intake motor
    private final Servo hoodServo;           // Adjustable hood servo

    // Odometry localizer
    private final TwoWheelLocalizer localizer;

    // Alliance & Hive configuration
    private final Alliance alliance;
    private HiveTarget activeHive = HiveTarget.HIVE_1;

    // Field coordinates for Hives 1 and 2 (Adjust these based on your field layout in inches)
    private static final double RED_HIVE_1_X = 144.0;
    private static final double RED_HIVE_1_Y = 36.0;
    private static final double RED_HIVE_2_X = 144.0;
    private static final double RED_HIVE_2_Y = 108.0;

    private static final double BLUE_HIVE_1_X = 0.0;
    private static final double BLUE_HIVE_1_Y = 36.0;
    private static final double BLUE_HIVE_2_X = 0.0;
    private static final double BLUE_HIVE_2_Y = 108.0;

    // Linear equation constants for hood adjustment based on distance: position = m * distance + c
    private double hoodSlope = 0.005;
    private double hoodIntercept = 0.1;

    // Preset power constants for Pollen vs Nectar & Intake
    public static final double POLLEN_POWER = 0.85;
    public static final double NECTAR_POWER = 0.70;
    public static final double INTAKE_POWER = 0.80;

    public subsystems(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Initialize Odometry Localizer and set TeleOp starting pose from RobotState (end of Auto)
        localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        localizer.setPose(RobotState.teleopStartingPose);

        // Initialize Motors
        shooter1Pollen = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2Nectar = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Set run modes
        shooter1Pollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2Nectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions
        shooter1Pollen.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2Nectar.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize Hood Servo
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void updateOdometry() {
        if (localizer != null) {
            localizer.update();
        }
    }

    public Pose getCurrentPose() {
        if (localizer != null) {
            return localizer.pose();
        }
        return new Pose(0, 0, 0);
    }

    public HiveTarget getActiveHive() {
        return activeHive;
    }

    /**
     * Automatically determines which hive the robot is closer to and calculates the distance.
     */
    public double getDistanceFromHive() {
        Pose pose = getCurrentPose();
        double target1X = (alliance == Alliance.RED) ? RED_HIVE_1_X : BLUE_HIVE_1_X;
        double target1Y = (alliance == Alliance.RED) ? RED_HIVE_1_Y : BLUE_HIVE_1_Y;
        double target2X = (alliance == Alliance.RED) ? RED_HIVE_2_X : BLUE_HIVE_2_X;
        double target2Y = (alliance == Alliance.RED) ? RED_HIVE_2_Y : BLUE_HIVE_2_Y;

        double dist1 = Math.hypot(target1X - pose.x(), target1Y - pose.y());
        double dist2 = Math.hypot(target2X - pose.x(), target2Y - pose.y());

        // Automatically switch active hive to whichever one is closer
        if (dist1 <= dist2) {
            activeHive = HiveTarget.HIVE_1;
            return dist1;
        } else {
            activeHive = HiveTarget.HIVE_2;
            return dist2;
        }
    }

    public void updateHoodAutomatically() {
        double distance = getDistanceFromHive(); // Automatically computes distance and updates activeHive
        double servoPosition = (hoodSlope * distance) + hoodIntercept;
        servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));
        hoodServo.setPosition(servoPosition);
    }

    public void setHoodPosition(double position) {
        double clamped = Math.max(0.0, Math.min(1.0, position));
        hoodServo.setPosition(clamped);
    }

    // Shooter Controls
    public void startShooter1Pollen() {
        shooter1Pollen.setPower(POLLEN_POWER);
        shooter2Nectar.setPower(0.0);
    }

    public void startShooter2Nectar() {
        shooter2Nectar.setPower(NECTAR_POWER);
        shooter1Pollen.setPower(0.0);
    }

    public void stopShooters() {
        shooter1Pollen.setPower(0.0);
        shooter2Nectar.setPower(0.0);
    }

    // Intake Controls
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void intakeIn() {
        intakeMotor.setPower(INTAKE_POWER);
    }

    public void intakeOut() {
        intakeMotor.setPower(-INTAKE_POWER);
    }

    public void stopIntake() {
        intakeMotor.setPower(0.0);
    }

    public void setHoodLinearParameters(double slope, double intercept) {
        this.hoodSlope = slope;
        this.hoodIntercept = intercept;
    }

    public double getHoodSlope() { return hoodSlope; }
    public double getHoodIntercept() { return hoodIntercept; }
    public double getHoodCurrentPosition() { return hoodServo.getPosition(); }
}
