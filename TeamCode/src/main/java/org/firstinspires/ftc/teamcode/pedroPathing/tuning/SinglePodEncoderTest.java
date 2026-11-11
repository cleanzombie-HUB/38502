package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Standalone OpMode to test a single odometry pod (encoder deadwheel).
 * <p>
 * Plug the single pod into any encoder port on the Control Hub or Expansion Hub.
 * <p>
 * Live Telemetry Features:
 * - Scans ALL configured motor encoder ports in your active hardware configuration.
 * - Displays absolute raw encoder ticks directly from hardware.
 * - Auto-detects active encoder movement.
 * - Displays calculated distance (in & cm), wheel rotations, velocity, and roll direction.
 * <p>
 * Controls:
 * - Gamepad 1 A / Cross: Zero out baseline & reset distance / peak tracking
 * - Gamepad 1 D-Pad Up / Down: Select focused encoder port
 * - Gamepad 1 Y / Triangle: Cycle wheel diameter preset (38mm, 35mm, 48mm, 2.0")
 */
@TeleOp(name = "Single Pod Encoder Test", group = "Pedro Pathing Tuning")
public class SinglePodEncoderTest extends LinearOpMode {

    private static class EncoderPort {
        String name;
        DcMotor motor;
        int baseline = 0;
        int lastPos = 0;

        EncoderPort(String name, DcMotor motor) {
            this.name = name;
            this.motor = motor;
            try {
                // Ensure motor mode allows encoder reading
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.baseline = motor.getCurrentPosition();
                this.lastPos = this.baseline;
            } catch (Exception ignored) {
            }
        }

        int getAbsoluteRawTicks() {
            try {
                return motor.getCurrentPosition();
            } catch (Exception e) {
                return 0;
            }
        }

        int getRelativeTicks() {
            return getAbsoluteRawTicks() - baseline;
        }

        void resetBaseline() {
            try {
                this.baseline = motor.getCurrentPosition();
                this.lastPos = 0;
            } catch (Exception ignored) {
            }
        }
    }

    // Wheel diameter presets in inches
    private final double[] WHEEL_DIAMETER_PRESETS = {38.0 / 25.4, 1.37795, 1.88976, 2.0};
    private final String[] WHEEL_PRESET_NAMES = {"38mm (1.496 in) [Default]", "35mm (1.378 in)", "48mm (1.890 in)", "2.0 inch (50.8 mm)"};
    private int presetIndex = 0;

    private boolean lastAState = false;
    private boolean lastYState = false;
    private boolean lastDUpState = false;
    private boolean lastDDownState = false;

    private int selectedPortIndex = 0;

    @Override
    public void runOpMode() {
        List<EncoderPort> ports = new ArrayList<>();

        // 1. Discover all DcMotor devices currently registered in hardwareMap
        try {
            for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
                try {
                    ports.add(new EncoderPort(entry.getKey(), entry.getValue()));
                } catch (Exception ignored) {
                }
            }
        } catch (Exception ignored) {
        }

        // 2. Fallback check for common standard names if entrySet is empty
        if (ports.isEmpty()) {
            String[] commonNames = {
                    "leftFront", "rightFront", "frontLeft", "backLeft", "frontRight", "backRight",
                    "shooter1", "shooter2", "intake", "xPod", "yPod"
            };
            for (String name : commonNames) {
                try {
                    if (hardwareMap.dcMotor.contains(name)) {
                        ports.add(new EncoderPort(name, hardwareMap.dcMotor.get(name)));
                    }
                } catch (Exception ignored) {
                }
            }
        }

        telemetry.addLine("=== INIT: HARDWARE MAP ENCODER DISCOVERY ===");
        telemetry.addData("Status", ports.isEmpty() ? "NO MOTORS FOUND IN CONFIG!" : "READY");
        telemetry.addData("Total Configured Motor Ports", ports.size());

        if (ports.isEmpty()) {
            telemetry.addLine("\n[ERROR] No motors were found in your active Robot Configuration!");
            telemetry.addLine("Go to Driver Station -> Configuration -> Control Hub -> Motors");
            telemetry.addLine("Ensure your motor ports (e.g., leftFront, rightFront) are configured.");
        } else {
            for (EncoderPort p : ports) {
                telemetry.addData("Registered Port", "%s (Raw Pos: %d)", p.name, p.getAbsoluteRawTicks());
            }
        }
        telemetry.update();

        waitForStart();

        // Reset all baselines on START
        for (EncoderPort p : ports) {
            p.resetBaseline();
        }

        ElapsedTime timer = new ElapsedTime();
        double lastTimeSec = timer.seconds();

        int activePortIndex = 0;
        int peakPositiveTicks = 0;
        int peakNegativeTicks = 0;

        while (opModeIsActive() && !isStopRequested()) {
            double currentTimeSec = timer.seconds();
            double dt = Math.max(currentTimeSec - lastTimeSec, 0.001);
            lastTimeSec = currentTimeSec;

            // Handle Gamepad A: Zero all baselines and peaks
            if (gamepad1.a && !lastAState) {
                for (EncoderPort p : ports) {
                    p.resetBaseline();
                }
                peakPositiveTicks = 0;
                peakNegativeTicks = 0;
            }
            lastAState = gamepad1.a;

            // Handle Gamepad Y: Cycle wheel diameter preset
            if (gamepad1.y && !lastYState) {
                presetIndex = (presetIndex + 1) % WHEEL_DIAMETER_PRESETS.length;
            }
            lastYState = gamepad1.y;

            // Handle Gamepad D-Pad: Cycle selected port
            if (!ports.isEmpty()) {
                if (gamepad1.dpad_up && !lastDUpState) {
                    selectedPortIndex = (selectedPortIndex - 1 + ports.size()) % ports.size();
                }
                if (gamepad1.dpad_down && !lastDDownState) {
                    selectedPortIndex = (selectedPortIndex + 1) % ports.size();
                }
            }
            lastDUpState = gamepad1.dpad_up;
            lastDDownState = gamepad1.dpad_down;

            double currentWheelDiameter = WHEEL_DIAMETER_PRESETS[presetIndex];
            double ticksPerRev = Constants.TICKS_PER_REV;
            double ticksToInches = (Math.PI * currentWheelDiameter) / ticksPerRev;

            // Auto-detect port movement if user is sliding wheel
            for (int i = 0; i < ports.size(); i++) {
                EncoderPort p = ports.get(i);
                int absTicks = p.getAbsoluteRawTicks();
                if (Math.abs(absTicks - p.lastPos) > 3) {
                    activePortIndex = i;
                }
            }

            // Use selected port or fallback to auto-detected active port
            int targetPortIndex = (!ports.isEmpty()) ? selectedPortIndex : -1;
            if (targetPortIndex < 0 || targetPortIndex >= ports.size()) {
                targetPortIndex = activePortIndex;
            }

            // --- TELEMETRY DISPLAY ---
            telemetry.addLine("=== SINGLE ODOMETRY POD TEST ===");
            telemetry.addData("Wheel Diameter", "%s", WHEEL_PRESET_NAMES[presetIndex]);
            telemetry.addData("Ticks / Revolution", "%.0f Ticks (REV Through Bore)", ticksPerRev);

            if (!ports.isEmpty() && targetPortIndex >= 0 && targetPortIndex < ports.size()) {
                EncoderPort focusedPort = ports.get(targetPortIndex);
                int rawAbsTicks = focusedPort.getAbsoluteRawTicks();
                int currentTicks = focusedPort.getRelativeTicks();
                double deltaTicksInFrame = rawAbsTicks - focusedPort.lastPos;
                focusedPort.lastPos = rawAbsTicks;

                if (currentTicks > peakPositiveTicks) peakPositiveTicks = currentTicks;
                if (currentTicks < peakNegativeTicks) peakNegativeTicks = currentTicks;

                double currentInches = currentTicks * ticksToInches;
                double currentCm = currentInches * 2.54;
                double rotations = currentTicks / ticksPerRev;

                double ticksPerSec = deltaTicksInFrame / dt;
                double inchesPerSec = ticksPerSec * ticksToInches;

                String directionStr = "IDLE";
                if (ticksPerSec > 5) {
                    directionStr = "FORWARD (>>>)";
                } else if (ticksPerSec < -5) {
                    directionStr = "REVERSE (<<<)";
                }

                telemetry.addLine("\n--- FOCUSED POD METRICS ---");
                telemetry.addData("Focused Motor Port Name", focusedPort.name);
                telemetry.addData("Absolute Hardware Raw Ticks", rawAbsTicks);
                telemetry.addData("Relative Session Ticks", currentTicks);
                telemetry.addData("Distance (Inches)", "%.3f in", currentInches);
                telemetry.addData("Distance (Centimeters)", "%.2f cm", currentCm);
                telemetry.addData("Wheel Rotations", "%.3f revs", rotations);
                telemetry.addData("Motion Direction", directionStr);

                telemetry.addLine("\n--- VELOCITY & PEAKS ---");
                telemetry.addData("Speed (Ticks/sec)", "%.1f ticks/s", ticksPerSec);
                telemetry.addData("Speed (Inches/sec)", "%.2f in/s", inchesPerSec);
                telemetry.addData("Peak Forward Ticks", peakPositiveTicks);
                telemetry.addData("Peak Reverse Ticks", peakNegativeTicks);

                telemetry.addLine("\n--- POD DIRECTION DIAGNOSTIC ---");
                if (currentTicks > 50) {
                    telemetry.addLine("Status: Wheel rolled FORWARD -> Ticks POSITIVE (CORRECT)");
                } else if (currentTicks < -50) {
                    telemetry.addLine("Status: Wheel rolled FORWARD -> Ticks NEGATIVE (INVERTED)");
                    telemetry.addLine("  -> In Constants, set Encoder.REVERSE for this pod!");
                } else {
                    telemetry.addLine("Status: Slide wheel forward by hand to verify tick direction.");
                }

            } else {
                telemetry.addLine("\n[ERROR] No motor ports found in hardwareMap!");
                telemetry.addLine("Ensure your Control Hub configuration has motor ports defined.");
            }

            telemetry.addLine("\n=== ALL DETECTED PORTS (LIVE ABSOLUTE RAW TICKS) ===");
            for (int i = 0; i < ports.size(); i++) {
                EncoderPort p = ports.get(i);
                String prefix = (i == targetPortIndex) ? "-> " : "   ";
                telemetry.addData(prefix + p.name, "Raw: %d | Rel: %d (%.2f in)",
                        p.getAbsoluteRawTicks(), p.getRelativeTicks(), p.getRelativeTicks() * ticksToInches);
            }

            telemetry.addLine("\n=== CONTROLS ===");
            telemetry.addLine("Press A: Zero Baseline & Peaks");
            telemetry.addLine("Press Y: Cycle Wheel Size Preset");
            telemetry.addLine("D-Pad Up/Down: Change Selected Port");

            telemetry.update();
        }
    }
}
