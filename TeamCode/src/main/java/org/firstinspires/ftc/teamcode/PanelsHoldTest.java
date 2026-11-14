package org.firstinspires.ftc.teamcode;

import com.bylazar.field.FieldManager;
import com.bylazar.field.FieldPresets;
import com.bylazar.field.PanelsField;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.BrakeTuning;

import java.util.ArrayDeque;
import java.util.Deque;

import static com.pedropathing.api.Paths.line;

@TeleOp(name = "Panels Hold Test")
public class PanelsHoldTest extends OpMode {

    private static final double END_PAUSE_SECONDS = 1.0;
    private static final int MAX_TRAIL_POINTS = 300;
    // Display-only offset: local (0, 0) appears at the center of the field.
    private static final double MAP_OFFSET = 72.0;

    private Follower follower;
    private boolean previousA;
    private Path outboundPath;
    private Path returnPath;
    private boolean running;
    private boolean outbound;
    private boolean pausing;
    private double targetHeading;
    private Double previousMaxVelocity;
    private Double previousMaxDeceleration;
    private double activeDistance;
    private final ElapsedTime pauseTimer = new ElapsedTime();
    private FieldManager field;
    private Pose lineStart;
    private Pose lineEnd;
    private final Deque<Pose> poseTrail = new ArrayDeque<>();
    private final ElapsedTime mapTimer = new ElapsedTime();

    @Override
    public void init() {
        previousMaxVelocity = Constants.foresightConfig.maxVelocityConstraint.get();
        previousMaxDeceleration = Constants.foresightConfig.maxDecelerationConstraint.get();
        BrakeTuning.apply();
        follower = Constants.create(hardwareMap);

        if (follower == null) {
            throw new IllegalStateException(
                "Constants.create() must return a Follower"
            );
        }

        follower.holdEnd.set(false);
        field = PanelsField.INSTANCE.getField();
        poseTrail.clear();
        mapTimer.reset();
        telemetry.addLine("A = Start back-and-forth line test using Panels pathDistance");
        telemetry.addData("Next distance (in)", BrakeTuning.nextPathDistance());
        telemetry.addLine("B = Stop; 1-second pause at each end");
        telemetry.addLine("Panels map: blue planned line, orange odometry trail, green robot");
        telemetry.update();
    }

    @Override
    public void loop() {
        BrakeTuning.apply();
        // Stop takes priority over starting, including simultaneous A/B presses.
        if (gamepad1.b) {
            running = false;
            pausing = false;
            follower.stop();
        } else if (gamepad1.a && !previousA && !running) {
            activeDistance = BrakeTuning.nextPathDistance();
            Pose start = follower.pose();
            targetHeading = start.heading();
            Pose end = new Pose(
                    start.x() + activeDistance * Math.cos(targetHeading),
                    start.y() + activeDistance * Math.sin(targetHeading),
                    targetHeading);
            lineStart = start;
            lineEnd = end;
            poseTrail.clear();
            poseTrail.addLast(start);
            // Keep the same endpoints and heading on every pass so drift is visible.
            outboundPath = makePath(start, end);
            returnPath = makePath(end, start);
            running = true;
            outbound = true;
            pausing = false;
            follower.follow(outboundPath);
        }

        previousA = gamepad1.a;

        follower.update();
        if (running && !follower.following()) {
            if (!pausing) {
                pausing = true;
                pauseTimer.reset();
            } else if (pauseTimer.seconds() >= END_PAUSE_SECONDS) {
                outbound = !outbound;
                follower.follow(outbound ? outboundPath : returnPath);
                pausing = false;
            }
        }
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("State", !running ? "Stopped" : pausing ? "Paused" : outbound ? "Outbound" : "Returning");
        telemetry.addData("Target heading", Math.toDegrees(targetHeading));
        telemetry.addData("Heading error (deg)",
                Math.toDegrees(Angle.normalizeSigned(targetHeading - follower.pose().heading())));
        telemetry.addData("Turn command", follower.debug().drivetrain().get("turn"));
        telemetry.addData("Forward command", follower.debug().drivetrain().get("forward"));
        telemetry.addData("Following", follower.following());
        telemetry.addData("Applied forward linear brake", BrakeTuning.appliedLinear());
        telemetry.addData("Applied forward quadratic brake", BrakeTuning.appliedQuadratic());
        telemetry.addData("Applied heading kP", BrakeTuning.appliedHeadingKP());
        telemetry.addData("Applied max velocity (in/s)", BrakeTuning.appliedMaxVelocity());
        telemetry.addData("Applied max deceleration (in/s²)", BrakeTuning.appliedMaxDeceleration());
        telemetry.addData("Active distance (in)", activeDistance);
        telemetry.addData("Next distance (in; B then A)", BrakeTuning.nextPathDistance());
        telemetry.addData("Brake tuning", BrakeTuning.status());
        telemetry.update();
        drawMap();
    }

    private void drawMap() {
        // Keep drawing/network work out of the fast follower update loop.
        if (mapTimer.seconds() < 0.1 || !field.getShouldUpdateCanvas()) return;
        mapTimer.reset();
        Pose pose = follower.pose();
        poseTrail.addLast(pose);
        while (poseTrail.size() > MAX_TRAIL_POINTS) poseTrail.removeFirst();

        field.getCanvas().reset();
        field.setOffsets(FieldPresets.INSTANCE.getPEDRO_PATHING());
        field.setStyle("transparent", "#3b82f6", 0.7);
        if (lineStart != null && lineEnd != null) {
            drawMapLine(lineStart.x(), lineStart.y(), lineEnd.x(), lineEnd.y());
            field.moveCursor(lineStart.x() + MAP_OFFSET, lineStart.y() + MAP_OFFSET);
            field.circle(1.5);
            field.moveCursor(lineEnd.x() + MAP_OFFSET, lineEnd.y() + MAP_OFFSET);
            field.circle(1.5);
        }

        field.setStyle("transparent", "#f97316", 0.4);
        Pose previous = null;
        for (Pose point : poseTrail) {
            if (previous != null) drawMapLine(previous.x(), previous.y(), point.x(), point.y());
            previous = point;
        }

        // Circular robot marker plus actual heading; this is estimated pose, not ground truth.
        field.setStyle("transparent", "#22c55e", 0.7);
        field.moveCursor(pose.x() + MAP_OFFSET, pose.y() + MAP_OFFSET);
        field.circle(9.0);
        drawMapLine(pose.x(), pose.y(),
                pose.x() + 12.0 * Math.cos(pose.heading()),
                pose.y() + 12.0 * Math.sin(pose.heading()));
        field.update();
    }

    private void drawMapLine(double x1, double y1, double x2, double y2) {
        field.moveCursor(x1 + MAP_OFFSET, y1 + MAP_OFFSET);
        field.line(x2 + MAP_OFFSET, y2 + MAP_OFFSET);
    }

    private Path makePath(Pose start, Pose end) {
        // Read the live configuration: a fixed path modifier would override live edits.
        return line(start, end).constant(targetHeading);
    }

    @Override
    public void stop() {
        running = false;
        if (follower != null) {
            follower.stop();
            follower.update();
        }
        if (previousMaxVelocity != null) {
            Constants.foresightConfig.maxVelocityConstraint.set(previousMaxVelocity);
        }
        if (previousMaxDeceleration != null) {
            Constants.foresightConfig.maxDecelerationConstraint.set(previousMaxDeceleration);
        }
    }
}
