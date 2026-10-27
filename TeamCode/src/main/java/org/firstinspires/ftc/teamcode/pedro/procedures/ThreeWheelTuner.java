package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.ThreeWheelConfig;
import com.pedropathing.revhub.localizers.ThreeWheelLocalizer;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class ThreeWheelTuner extends Procedure {

    private static String leftEncoderName = "br";
    private static String rightEncoderName = "bl";
    private static String strafeEncoderName = "fl";

    public ThreeWheelTuner() {
        super("Three Wheel Tuner", "Tune three odometry pods");
    }

    @Override
    public void run() throws InterruptedException {
        Inputs setup = inputs("Encoder Setup",
                "Set the motor ports that the three odometry encoders are plugged into.");
        Inputs.Field<String> leftEncoder = setup.s("Left Encoder Motor Name").withDefault("br");
        Inputs.Field<String> rightEncoder = setup.s("Right Encoder Motor Name").withDefault("bl");
        Inputs.Field<String> strafeEncoder = setup.s("Strafe Encoder Motor Name").withDefault("fl");
        awaitInputs(setup);
        leftEncoderName = leftEncoder.get();
        rightEncoderName = rightEncoder.get();
        strafeEncoderName = strafeEncoder.get();

        // Preset resolutions from the measured 48-inch translation tests.
        // Encoder directions are provisional until verified with the FINAL br/bl/fl mapping.
        // A changed hub port or motor direction can change the observed raw tick sign.
        // Skip resolution tests; retain pod offset and turn calibration stages.
        List<Double> left = List.of(336.9166666666667, Encoder.FORWARD);
        List<Double> right = List.of(335.6666666666667, Encoder.REVERSE);
        List<Double> strafe = List.of(339.2916666666667, Encoder.REVERSE);

        double leftTicksPerInch = left.get(0);
        double rightTicksPerInch = right.get(0);
        double strafeTicksPerInch = strafe.get(0);
        double forwardTicksPerInch = 2.0 / (1.0 / leftTicksPerInch + 1.0 / rightTicksPerInch);

        double forward = 1.0 / forwardTicksPerInch;
        double lateral = 1.0 / strafeTicksPerInch;

        List<Double> leftOffsets = runOpMode(new ThreeWheelOffsets(
                true, forward, lateral, left.get(1), right.get(1), strafe.get(1)));
        if (leftOffsets == null) {
            abort("Left stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.");
            return;
        }
        List<Double> rightOffsets = runOpMode(new ThreeWheelOffsets(
                false, forward, lateral, left.get(1), right.get(1), strafe.get(1)));

        if (rightOffsets == null) {
            abort("Right stage ended without parallel pod travel. Rotate 180 degrees CCW, then press Stop.");
            return;
        }
        ThreeWheelConfig config = config(true, forward, lateral,
                left.get(1), right.get(1), strafe.get(1));
        config.leftPodY.set(leftOffsets.get(0));
        config.rightPodY.set(rightOffsets.get(0));
        config.turnTicksToRadians.set(forward);

        Double turn = runOpMode(new ThreeWheelTurn(config));
        if (turn == null) {
            abort("Turn stage ended without positive rotation. Rotate 360 degrees CCW, then press Stop.");
            return;
        }
        // Measure perpendicular pod offset from raw encoder travel during one full rotation.
        Double strafeX = runOpMode(new ThreeWheelStrafeOffset(
                strafeEncoderName, lateral, strafe.get(1)));
        if (strafeX == null) {
            abort("Strafe offset stage failed: check for nonzero ticks and rotate 360 degrees CCW.");
            return;
        }

        result("leftEncoderName", leftEncoderName);
        result("rightEncoderName", rightEncoderName);
        result("strafeEncoderName", strafeEncoderName);
        result("leftPodY", leftOffsets.get(0));
        result("rightPodY", rightOffsets.get(0));
        result("strafePodX", strafeX);
        result("leftTicksPerInch", leftTicksPerInch);
        result("rightTicksPerInch", rightTicksPerInch);
        result("forwardTicksPerInch", forwardTicksPerInch);
        result("strafeTicksPerInch", strafeTicksPerInch);
        result("forwardTicksToInches", forward);
        result("strafeTicksToInches", lateral);
        result("turnTicksToRadians", turn);
        result("leftEncoderDirection", direction(left.get(1)));
        result("rightEncoderDirection", direction(right.get(1)));
        result("strafeEncoderDirection", direction(strafe.get(1)));

        code(Language.JAVA,
                "public static ThreeWheelConfig localizerConfig = new ThreeWheelConfig(c -> {\n" +
                        "    c.leftEncoderName.set(\"" + leftEncoderName + "\");\n" +
                        "    c.rightEncoderName.set(\"" + rightEncoderName + "\");\n" +
                        "    c.strafeEncoderName.set(\"" + strafeEncoderName + "\");\n" +
                        "    c.leftPodY.set(" + leftOffsets.get(0) + ");\n" +
                        "    c.rightPodY.set(" + rightOffsets.get(0) + ");\n" +
                        "    c.strafePodX.set(" + strafeX + ");\n" +
                        "    c.forwardTicksToInches.set(" + forward + ");\n" +
                        "    c.strafeTicksToInches.set(" + lateral + ");\n" +
                        "    c.turnTicksToRadians.set(" + turn + ");\n" +
                        "    c.leftEncoderDirection.set(" + direction(left.get(1)) + ");\n" +
                        "    c.rightEncoderDirection.set(" + direction(right.get(1)) + ");\n" +
                        "    c.strafeEncoderDirection.set(" + direction(strafe.get(1)) + ");\n" +
                        "});");
    }

    private List<Double> measure(String pod, double distance) throws InterruptedException {
        List<Double> measured = runOpMode(new ThreeWheelResolution(pod, distance));
        if (measured == null) {
            abort(pod + " stage ended without a nonzero measurement. Check the displayed ticks, complete the push, then press Stop.");
            return null;
        }
        return measured;
    }

    static String direction(double direction) {
        return direction == Encoder.REVERSE ? "Encoder.REVERSE" : "Encoder.FORWARD";
    }

    static ThreeWheelConfig config(boolean left, double forward, double strafe,
                                   double leftDirection, double rightDirection, double strafeDirection) {
        return new ThreeWheelConfig(c -> {
            c.leftEncoderName.set(leftEncoderName);
            c.rightEncoderName.set(rightEncoderName);
            c.strafeEncoderName.set(strafeEncoderName);
            c.leftPodY.set(left ? 0.0 : 1.0);
            c.rightPodY.set(left ? -1.0 : 0.0);
            c.strafePodX.set(0.0);
            c.forwardTicksToInches.set(forward);
            c.strafeTicksToInches.set(strafe);
            c.turnTicksToRadians.set(0.0);
            c.leftEncoderDirection.set(leftDirection);
            c.rightEncoderDirection.set(rightDirection);
            c.strafeEncoderDirection.set(strafeDirection);
        });
    }

    static ThreeWheelLocalizer localizer(HardwareMap map, ThreeWheelConfig config) {
        for (LynxModule hub : map.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        for (String name : new String[]{"fl", "bl", "fr", "br"}) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setDirection(
                    name.equals("fl") || name.equals("bl")
                            ? DcMotorSimple.Direction.REVERSE
                            : DcMotorSimple.Direction.FORWARD
            );
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.FLOAT
            );
        }

        return new ThreeWheelLocalizer(map, config);
    }
}

class ThreeWheelResolution extends TuningOpMode<List<Double>> {

    String pod;
    double distance;

    ThreeWheelResolution(String pod, double distance) {
        super(pod + " Encoder Resolution and Direction",
                "After Start, push the robot " + (pod.equals("Strafe") ? "left " : "forward ") +
                        distance + " inches exactly without turning. Stop moving, press Stop to save this measurement.", true);
        this.pod = pod;
        this.distance = distance;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        ThreeWheelConfig config = ThreeWheelTuner.config(!pod.equals("Right"), 1.0, 1.0,
                Encoder.FORWARD, Encoder.FORWARD, Encoder.FORWARD);
        ThreeWheelLocalizer localizer = ThreeWheelTuner.localizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        Pose position = null;

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            position = localizer.pose();
        }

        if (position == null) {
            return null;
        }
        double movement = pod.equals("Strafe") ? position.y() : position.x();
        if (movement == 0.0) {
            return null;
        }
        return List.of(Math.abs(movement / distance), movement < 0 ? Encoder.REVERSE : Encoder.FORWARD);
    }
}

class ThreeWheelOffsets extends TuningOpMode<List<Double>> {

    boolean left;
    double forward;
    double strafe;
    double leftDirection;
    double rightDirection;
    double strafeDirection;

    ThreeWheelOffsets(boolean left, double forward, double strafe,
                      double leftDirection, double rightDirection, double strafeDirection) {
        super((left ? "Left" : "Right") + " Pod Offset Identification",
                "After Start, rotate exactly 180 degrees counterclockwise about the robot center. " +
                        "Keep that center fixed. Stop moving, press Stop to save this measurement.", true);
        this.left = left;
        this.forward = forward;
        this.strafe = strafe;
        this.leftDirection = leftDirection;
        this.rightDirection = rightDirection;
        this.strafeDirection = strafeDirection;
    }

    @Override
    protected List<Double> runTuningOpMode() {
        ThreeWheelConfig config = ThreeWheelTuner.config(left, forward, strafe,
                leftDirection, rightDirection, strafeDirection);
        ThreeWheelLocalizer localizer = ThreeWheelTuner.localizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();
        Pose position = null;

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            position = localizer.pose();
        }

        if (position == null) {
            return null;
        }
        return List.of(-position.x() / Math.PI, position.y() / Math.PI);
    }
}

class ThreeWheelTurn extends TuningOpMode<Double> {

    ThreeWheelConfig config;

    ThreeWheelTurn(ThreeWheelConfig config) {
        super("Turn Multiplier Identification",
                "After Start, rotate exactly 360 degrees counterclockwise. " +
                        "Stop moving, press Stop to save this measurement.", true);
        this.config = config;
    }

    @Override
    protected Double runTuningOpMode() {
        ThreeWheelLocalizer localizer = ThreeWheelTuner.localizer(hardwareMap, config);
        localizer.setPose(new Pose(0, 0));
        localizer.update();
        double startHeading = localizer.getTotalHeading();
        Double heading = null;

        waitForStart();
        while (!isStopRequested()) {
            localizer.update();
            heading = localizer.getTotalHeading();
        }

        if (heading == null || heading <= startHeading) {
            return null;
        }
        return config.turnTicksToRadians.get() * (2.0 * Math.PI) / (heading - startHeading);
    }
}




/** Calibrates the perpendicular pod's forward/backward (X) offset from raw ticks. */
class ThreeWheelStrafeOffset extends TuningOpMode<Double> {
    private final String encoderName;
    private final double inchesPerTick;
    private final double encoderDirection;

    ThreeWheelStrafeOffset(String encoderName, double inchesPerTick, double encoderDirection) {
        super("Strafe Pod X Offset Identification",
                "After Start, rotate exactly 360 degrees CCW about the robot center, " +
                "without translating. Press Stop to save the measurement.", true);
        this.encoderName = encoderName;
        this.inchesPerTick = inchesPerTick;
        this.encoderDirection = encoderDirection;
    }

    @Override
    protected Double runTuningOpMode() {
        // Apply the same motor-direction settings as the other tuner stages.
        // This matters because FTC motor direction can affect encoder sign.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        for (String name : new String[]{"fl", "bl", "fr", "br"}) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setPower(0);
            motor.setDirection(name.equals("fl") || name.equals("bl")
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, encoderName);
        waitForStart();
        int start = encoder.getCurrentPosition();
        int current = start;
        while (!isStopRequested()) {
            current = encoder.getCurrentPosition();
            telemetry.addData("Encoder", encoderName);
            telemetry.addData("Delta ticks", current - start);
            telemetry.update();
            idle();
        }
        int delta = current - start;
        if (delta == 0) return null;

        // A positive CCW angle is +2pi rad. Positive Y is left.
        // The wheel's signed lateral travel is +X_offset * angle.
        return delta * encoderDirection * inchesPerTick / (2.0 * Math.PI);
    }
}
