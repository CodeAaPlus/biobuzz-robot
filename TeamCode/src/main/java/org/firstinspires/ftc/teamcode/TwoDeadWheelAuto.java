package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Small first test of the standalone dead-wheel and IMU hardware class. */
@Autonomous(name = "Two Dead Wheel Auto", group = "Autonomous")
public class TwoDeadWheelAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TwoDeadWheelDrive robot = new TwoDeadWheelDrive(hardwareMap);

        telemetry.addLine("Forward 12 in at 20% power; IMU holds starting heading.");
        telemetry.addLine("Logo DOWN, USB RIGHT. Clear the area and keep STOP ready.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        try {
            String result = robot.driveForward(this, 12.0, 0.20, 3.0);
            robot.drive(0, 0.0, 0);
            telemetry.addData("Result", result);
            telemetry.update();
        } finally {
            robot.stop();
        }
    }
}
