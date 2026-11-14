package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Latch Calibration")
public class LatchCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo latch = hardwareMap.get(Servo.class, "latch");

        double position = 0.5;
        boolean lastUp = false;
        boolean lastDown = false;

        telemetry.addLine("Ready to calibrate latch");
        telemetry.update();

        waitForStart();

        latch.setPosition(position);

        while (opModeIsActive()) {

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            if (up && !lastUp) {
                position = Math.min(1.0, position + 0.01);
            }

            if (down && !lastDown) {
                position = Math.max(0.0, position - 0.01);
            }

            if (gamepad1.a) {
                position = 0.5;
            }

            lastUp = up;
            lastDown = down;

            latch.setPosition(position);

            telemetry.addData("Latch Position", "%.3f", position);
            telemetry.addLine("D-pad Up/Down: Adjust by 0.01");
            telemetry.addLine("A: Reset to 0.5");
            telemetry.update();

            sleep(20);
        }
    }
}
