package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(133.3382, 34.2624, 180);
    private final Pose point1 = poseFactory.of(109.5613, 9.2467, 180);
    private final Pose point1Control1 = poseFactory.of(105.6033, 32.7534, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1())
        );
    }


    @Override
    public void runOpMode() {
        Scheduler.reset();
        BrakeTuning.apply();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        if (isStopRequested()) return;
        BrakeTuning.apply();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            BrakeTuning.apply();
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Follower mode", follower.mode());
            telemetry.addData("Hold end enabled", follower.holdEnd.get());
            telemetry.addData("Turn command", follower.debug().drivetrain().get("turn"));
            telemetry.addData("Applied forward linear brake", BrakeTuning.appliedLinear());
            telemetry.addData("Applied forward quadratic brake", BrakeTuning.appliedQuadratic());
            telemetry.addData("Applied heading kP", BrakeTuning.appliedHeadingKP());
            telemetry.addData("Applied max velocity (in/s)", BrakeTuning.appliedMaxVelocity());
            telemetry.addData("Applied max deceleration (in/s²)", BrakeTuning.appliedMaxDeceleration());
            telemetry.addData("Panels tuning", BrakeTuning.status());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

}
