package org.firstinspires.ftc.teamcode.helper;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private final DcMotorEx intakeMotor;       // Intake motor (floor pickup)
    private final DcMotorEx sortingMotor;      // Sorting motor (pushes/sorts balls into shooter)
    private final Servo gateServo;           // Servo gate blocking/releasing balls into flywheels
    private final Servo rampServo;           // Ramp servo lowering/raising intake ramp for flower

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

    // Servo Gate Position Constants
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN   = 0.6;

    // Ramp Servo Position Constants (90 degrees rotation difference)
    public static final double RAMP_UP   = 0.0; // Stowed up position
    public static final double RAMP_DOWN = 0.5; // Lowered down position for flower intake

    // Studica Maverick Encoder Specs:
    // Shooter 1 (Pollen): 1:1 6100 RPM Motor (112.0 4x quadrature ticks/rev)
    public static final double SHOOTER1_TICKS_PER_REV = 112.0;

    // Shooter 2 (Nectar): 1:1 6100 RPM Motor (112.0 4x quadrature ticks/rev)
    public static final double SHOOTER2_TICKS_PER_REV = 112.0;

    // Preset Target RPM constants
    public static final double POLLEN_RPM = 3800.0; // Shooter 1 (Max 6100 RPM)
    public static final double NECTAR_RPM = 4400.0; // Shooter 2 (Max 6100 RPM)

    // Motor Power Constants (Full Power = 1.0)
    public static final double INTAKE_POWER  = 1.0;
    public static final double SORTING_POWER = 1.0;

    public subsystems(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;

        // Initialize Odometry Localizer and set TeleOp starting pose from RobotState (end of Auto)
        localizer = new TwoWheelLocalizer(hardwareMap, Constants.createTwoWheelConfig());
        localizer.setPose(RobotState.teleopStartingPose);

        // Initialize Motors
        shooter1Pollen = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2Nectar = hardwareMap.get(DcMotorEx.class, "shooter2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        sortingMotor = hardwareMap.get(DcMotorEx.class, "sorting");

        // Set run modes
        shooter1Pollen.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2Nectar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sortingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Tuned Feedforward & Proportional Gain (F=13.5, P=12.0, I=0.0)
        PIDFCoefficients flywheelPidf = new PIDFCoefficients(12.0, 0.0, 0.0, 13.5);
        shooter1Pollen.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);
        shooter2Nectar.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);

        // Set motor directions
        shooter1Pollen.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2Nectar.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        sortingMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize Servos
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED); // Default gate is CLOSED

        rampServo = hardwareMap.get(Servo.class, "rampServo");
        rampServo.setPosition(RAMP_UP); // Default ramp is UP
    }

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
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

    // Servo Gate Controls
    public void openGate() {
        gateServo.setPosition(GATE_OPEN);
    }

    public void closeGate() {
        gateServo.setPosition(GATE_CLOSED);
    }

    public void setGatePosition(double position) {
        gateServo.setPosition(position);
    }

    // Ramp Servo Controls (Flower Intake)
    public void lowerRamp() {
        rampServo.setPosition(RAMP_DOWN);
    }

    public void raiseRamp() {
        rampServo.setPosition(RAMP_UP);
    }

    public void setRampPosition(double position) {
        rampServo.setPosition(position);
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

    // Shooter Controls (RPM Velocity Control)
    public void startShooter1Pollen() {
        shooter1Pollen.setVelocity(rpmToTicksPerSec(POLLEN_RPM, SHOOTER1_TICKS_PER_REV));
        shooter2Nectar.setVelocity(0.0);
    }

    public void startShooter2Nectar() {
        shooter2Nectar.setVelocity(rpmToTicksPerSec(NECTAR_RPM, SHOOTER2_TICKS_PER_REV));
        shooter1Pollen.setVelocity(0.0);
    }

    public void stopShooters() {
        shooter1Pollen.setVelocity(0.0);
        shooter2Nectar.setVelocity(0.0);
        closeGate();
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

    // Sorting Controls
    public void setSortingPower(double power) {
        sortingMotor.setPower(power);
    }

    public void sortingIn() {
        sortingMotor.setPower(SORTING_POWER);
    }

    public void sortingOut() {
        sortingMotor.setPower(-SORTING_POWER);
    }

    public void stopSorting() {
        sortingMotor.setPower(0.0);
    }

    public double getGateCurrentPosition() { return gateServo.getPosition(); }
    public double getRampCurrentPosition() { return rampServo.getPosition(); }
}
