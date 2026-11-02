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

    // Current selected shooting position (Default: CLOSE)
    private ShootingPosition currentPosition = ShootingPosition.CLOSE;

    // Fixed Hood Angle Position Constant
    public static final double HOOD_FIXED_POSITION = 0.40;

    // Preset Power / Speed Constants for Pollen (Shooter 1)
    public static final double POLLEN_CLOSE_POWER = 0.70;
    public static final double POLLEN_FAR_POWER = 0.90;

    // Preset Power / Speed Constants for Nectar (Shooter 2)
    public static final double NECTAR_CLOSE_POWER = 0.60;
    public static final double NECTAR_FAR_POWER = 0.80;

    // Intake Power Constant
    public static final double INTAKE_POWER = 0.80;

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

        // Initialize Hood Servo and lock to fixed position
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        hoodServo.setPosition(HOOD_FIXED_POSITION);
    }

    /**
     * Select shooting position preset (CLOSE vs FAR)
     */
    public void setShootingPosition(ShootingPosition position) {
        this.currentPosition = position;
    }

    public ShootingPosition getShootingPosition() {
        return currentPosition;
    }

    /**
     * Start Shooter 1 (Pollen) based on preset position
     */
    public void startShooter1Pollen() {
        double power = (currentPosition == ShootingPosition.CLOSE) ? POLLEN_CLOSE_POWER : POLLEN_FAR_POWER;
        shooter1Pollen.setPower(power);
        shooter2Nectar.setPower(0.0);
    }

    /**
     * Start Shooter 2 (Nectar) based on preset position
     */
    public void startShooter2Nectar() {
        double power = (currentPosition == ShootingPosition.CLOSE) ? NECTAR_CLOSE_POWER : NECTAR_FAR_POWER;
        shooter2Nectar.setPower(power);
        shooter1Pollen.setPower(0.0);
    }

    /**
     * Stop both shooters
     */
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

    // Set fixed hood position
    public void setFixedHoodPosition(double position) {
        hoodServo.setPosition(position);
    }

    public double getHoodCurrentPosition() {
        return hoodServo.getPosition();
    }
}
