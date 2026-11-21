package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class rawSubsystem {

    public enum ShootingPosition {
        CLOSE,
        FAR
    }

    public enum GamePiece {
        POLLEN,
        NECTAR
    }

    // Hardware components
    private final DcMotorEx shooter1Pollen; // Shooter 1: Pollen (Biobuzz)
    private final DcMotorEx shooter2Nectar;  // Shooter 2: Nectar
    private final DcMotorEx intakeMotor;       // Intake motor (floor pickup)
    private final DcMotorEx sortingMotor;      // Sorting motor (pushes/sorts balls into shooter)
    private final Servo gateServo;           // Servo gate blocking/releasing balls into flywheels
    private final Servo rampServo;           // Ramp servo lowering/raising intake ramp for flower

    // Current selected shooting position (Default: CLOSE)
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // Studica Maverick Encoder Specs:
    // Shooter 1 (Pollen): 3.7:1 Motor Geared UP 2:1 to Flywheel (3298 Max RPM, 51.8 ticks per rev)
    public static final double SHOOTER1_TICKS_PER_REV = 51.8;

    // Shooter 2 (Nectar): 3.7:1 Motor Geared UP 2:1 to Flywheel (3298 Max RPM, 51.8 ticks per rev)
    public static final double SHOOTER2_TICKS_PER_REV = 51.8;

    // Servo Gate Position Constants (Adjust based on mechanical setup)
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN   = 0.6;

    // Ramp Servo Position Constants (90 degrees rotation difference)
    public static final double RAMP_UP   = 0.0; // Stowed up position
    public static final double RAMP_DOWN = 0.5; // Lowered down position for flower intake

    // Preset Target RPM Constants for Pollen (Shooter 1: Max 3298 RPM, realistic cap ~2500 RPM)
    public static final double POLLEN_CLOSE_RPM = 1800.0;
    public static final double POLLEN_FAR_RPM   = 2500.0;

    // Preset Target RPM Constants for Nectar (Shooter 2: Max 3298 RPM, realistic cap ~2500 RPM)
    public static final double NECTAR_CLOSE_RPM = 1800.0;
    public static final double NECTAR_FAR_RPM   = 2500.0;

    // Motor Power Constants (Intake = 100%, Sorting = 40% for controlled feeding)
    public static final double INTAKE_POWER  = 1.0;
    public static final double SORTING_POWER = 0.40; // Controlled sorting power gives flywheel time to spin up

    public rawSubsystem(HardwareMap hardwareMap) {
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

    public void setShootingPosition(ShootingPosition position) {
        this.currentPosition = position;
    }

    public ShootingPosition getShootingPosition() {
        return currentPosition;
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
     * Start Shooter 1 (Pollen) using target RPM preset
     */
    public void startShooter1Pollen() {
        double targetRpm = (currentPosition == ShootingPosition.CLOSE) ? POLLEN_CLOSE_RPM : POLLEN_FAR_RPM;
        shooter1Pollen.setVelocity(rpmToTicksPerSec(targetRpm, SHOOTER1_TICKS_PER_REV));
        shooter2Nectar.setVelocity(0.0);
    }

    /**
     * Start Shooter 2 (Nectar) using target RPM preset
     */
    public void startShooter2Nectar() {
        double targetRpm = (currentPosition == ShootingPosition.CLOSE) ? NECTAR_CLOSE_RPM : NECTAR_FAR_RPM;
        shooter2Nectar.setVelocity(rpmToTicksPerSec(targetRpm, SHOOTER2_TICKS_PER_REV));
        shooter1Pollen.setVelocity(0.0);
    }

    /**
     * Stop both shooters and close gate
     */
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

    public double getGateCurrentPosition() {
        return gateServo.getPosition();
    }

    public double getRampCurrentPosition() {
        return rampServo.getPosition();
    }
}
