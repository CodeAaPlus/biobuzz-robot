
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Three Wheel Encoder Test")
public class ThreeWheelEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Encoder port names from ThreeWheelTuner

        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "br");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx strafe = hardwareMap.get(DcMotorEx.class, "fl");


        // Read existing positions without resetting encoders
        int leftStart = left.getCurrentPosition();
        int rightStart = right.getCurrentPosition();
        int strafeStart = strafe.getCurrentPosition();

        telemetry.addLine("Three encoders initialized!");
        telemetry.addLine("Push robot by hand after Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int leftTicks = left.getCurrentPosition() - leftStart;
            int rightTicks = right.getCurrentPosition() - rightStart;
            int strafeTicks = strafe.getCurrentPosition() - strafeStart;

            telemetry.addData("LEFT (fr)", leftTicks);
            telemetry.addData("RIGHT (bl)", rightTicks);
            telemetry.addData("STRAFE (fl)", strafeTicks);

            telemetry.addLine("------------------");
            telemetry.addLine("Forward: left/right should change");
            telemetry.addLine("Sideways: strafe should change");
            telemetry.addLine("Rotate: all may change");

            telemetry.update();
            idle();
        }
    }
}
