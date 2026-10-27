package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "Panels Hold Test")
public class PanelsHoldTest extends OpMode {

    private Follower follower;
    private boolean previousA;
    private boolean previousB;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        if (follower == null) {
            throw new IllegalStateException(
                "Constants.create() must return a Follower"
            );
        }

        telemetry.addLine("A = Begin holding");
        telemetry.addLine("B = Stop holding");
        telemetry.update();
    }

    @Override
    public void loop() {
        PanelsForesightTuning.apply();

        if (gamepad1.a && !previousA) {
            follower.hold(follower.pose(), true);
        }

        if (gamepad1.b && !previousB) {
            follower.stop();
        }

        previousA = gamepad1.a;
        previousB = gamepad1.b;

        follower.update();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Holding", follower.holding());
        telemetry.addData("Heading kP",
            PanelsForesightTuning.headingKP);
        telemetry.addData("Forward Secondary kP",
            PanelsForesightTuning.forwardSecondaryKP);
        telemetry.addData("Strafe Secondary kP",
            PanelsForesightTuning.strafeSecondaryKP);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }
}
