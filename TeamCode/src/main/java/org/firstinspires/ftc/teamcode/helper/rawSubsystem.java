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
    private final DcMotorEx intakeMotor;       // Intake motor
    private final Servo hoodServo;           // Hood servo held at a fixed angle
    private final Servo gateServo;           // Servo gate blocking/releasing balls into flywheels

    // Current selected shooting position (Default: CLOSE)
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // Motor Encoder Specs (Adjust this to match your flywheel motor's encoder ticks per revolution)
    public static final double TICKS_PER_REV = 537.7; // e.g. 537.7 for GoBilda 312 RPM motor

    // Fixed Hood Angle Position Constant
    public static final double HOOD_FIXED_POSITION = 0.40;

    // Servo Gate Position Constants (Adjust based on mechanical setup)
    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN   = 0.6;

    // Preset Target RPM Constants for Pollen (Shooter 1) - Lighter piece
    public static final double POLLEN_CLOSE_RPM = 3400.0;
    public static final double POLLEN_FAR_RPM   = 4400.0;

    // Preset Target RPM Constants for Nectar (Shooter 2) - Heavier piece
    public static final double NECTAR_CLOSE_RPM = 4000.0;
    public static final double NECTAR_FAR_RPM   = 5000.0;

    // Intake Power Constant (Full Power = 1.0)
    public static final double INTAKE_POWER = 1.0;

    public rawSubsystem(HardwareMap hardwareMap) {
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

        // Initialize Servos
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(HOOD_FIXED_POSITION);

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(GATE_CLOSED); // Default gate is CLOSED
    }

    /**
     * Converts target RPM to encoder ticks per second
     */
    private double rpmToTicksPerSec(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
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

    /**
     * Start Shooter 1 (Pollen) using target RPM preset
     */
    public void startShooter1Pollen() {
        double targetRpm = (currentPosition == ShootingPosition.CLOSE) ? POLLEN_CLOSE_RPM : POLLEN_FAR_RPM;
        shooter1Pollen.setVelocity(rpmToTicksPerSec(targetRpm));
        shooter2Nectar.setVelocity(0.0);
    }

    /**
     * Start Shooter 2 (Nectar) using target RPM preset
     */
    public void startShooter2Nectar() {
        double targetRpm = (currentPosition == ShootingPosition.CLOSE) ? NECTAR_CLOSE_RPM : NECTAR_FAR_RPM;
        shooter2Nectar.setVelocity(rpmToTicksPerSec(targetRpm));
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

    public void setFixedHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodCurrentPosition() {
        return hoodServo.getPosition();
    }

    public double getGateCurrentPosition() {
        return gateServo.getPosition();
    }
}
