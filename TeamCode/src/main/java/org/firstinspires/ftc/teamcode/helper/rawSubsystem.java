package org.firstinspires.ftc.teamcode.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private final Servo rampServoLeft;       // Left ramp servo
    private final Servo rampServoRight;      // Right ramp servo (reversed orientation)

    // Current selected shooting position (Default: CLOSE)
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // Studica Maverick Encoder Specs:
    // Shooter 1 (Pollen): 1:1 6100 RPM Motor (112.0 4x quadrature ticks/rev)
    public static final double SHOOTER1_TICKS_PER_REV = 24.0;

    // Shooter 2 (Nectar): 1:1 6100 RPM Motor (112.0 4x quadrature ticks/rev)
    public static final double SHOOTER2_TICKS_PER_REV = 24.0;

    // Studica 75002 Smart Servo Total Motion Range in Degrees (Default 270.0°)
    public static final double RAMP_SERVO_RANGE_DEGREES = 270.0;

    // Ramp Angles in DEGREES
    public static final double RAMP_UP_DEGREES   = 0.0;   // 0° Stowed up position
    public static final double RAMP_DOWN_DEGREES = 200.0; // 200° Lowered down position for flower intake

    // Preset Target RPM Constants for Pollen (Shooter 1: Max 6100 RPM)
    public static final double POLLEN_CLOSE_RPM = 2500.0;
    public static final double POLLEN_FAR_RPM   = 1800.0;

    // Preset Target RPM Constants for Nectar (Shooter 2: Max 6100 RPM)
    public static final double NECTAR_CLOSE_RPM = 3600.0;
    public static final double NECTAR_FAR_RPM   = 4800.0;

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

        // Tuned Feedforward & Proportional Gain (F=13.5, P=12.0, I=0.0)
        PIDFCoefficients flywheelPidf = new PIDFCoefficients(12.0, 0.0, 0.0, 13.5);
        shooter1Pollen.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);
        shooter2Nectar.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheelPidf);

        // Set motor directions
        shooter1Pollen.setDirection(DcMotorEx.Direction.REVERSE);
        shooter2Nectar.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        sortingMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // Initialize Dual Ramp Servos ("rampLeft" and "rampRight")
        rampServoLeft = hardwareMap.get(Servo.class, "rampLeft");
        rampServoRight = hardwareMap.get(Servo.class, "rampRight");

        rampServoLeft.setDirection(Servo.Direction.FORWARD);
        rampServoRight.setDirection(Servo.Direction.REVERSE);

        raiseRamp();
    }

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    /**
     * Converts angle in DEGREES to FTC Servo position (0.0 to 1.0)
     */
    private double degreesToServoPosition(double degrees) {
        return Math.max(0.0, Math.min(1.0, degrees / RAMP_SERVO_RANGE_DEGREES));
    }

    public void setShootingPosition(ShootingPosition position) {
        this.currentPosition = position;
    }

    public ShootingPosition getShootingPosition() {
        return currentPosition;
    }

    // Ramp Servo Controls (Specified directly in DEGREES)
    public void lowerRamp() {
        setRampAngle(RAMP_DOWN_DEGREES); // 200°
    }

    public void raiseRamp() {
        setRampAngle(RAMP_UP_DEGREES); // 0°
    }

    public void setRampAngle(double degrees) {
        double pos = degreesToServoPosition(degrees);
        rampServoLeft.setPosition(pos);
        rampServoRight.setPosition(pos);
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
     * Stop both shooters
     */
    public void stopShooters() {
        shooter1Pollen.setVelocity(0.0);
        shooter2Nectar.setVelocity(0.0);
    }

    // Autonomous Helper Methods
    public void shootPollenAuto() {
        startShooter1Pollen();
        sortingIn();
    }

    public void stopShooterAuto() {
        stopShooters();
        stopSorting();
    }

    public void intakeAuto() {
        lowerRamp();
        intakeIn();
    }

    public void stopIntakeAuto() {
        stopIntake();
        raiseRamp();
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

    public double getRampCurrentPosition() {
        return rampServoLeft.getPosition();
    }

    public double getRampCurrentDegrees() {
        return rampServoLeft.getPosition() * RAMP_SERVO_RANGE_DEGREES;
    }
}
