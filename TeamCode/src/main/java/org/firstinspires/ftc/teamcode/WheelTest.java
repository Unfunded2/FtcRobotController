package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class WheelTest extends LinearOpMode {
    private DcMotor whatAmI;

    public void runOpMode() {
        whatAmI = hardwareMap.get(DcMotor.class, "whatAmI");

        waitForStart();
        whatAmI.setPower(-0.5);
        sleep(1000);
        whatAmI.setPower(0);
    }
}
