package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "GetHexPOS")
public class getHexPos extends LinearOpMode {
    public DcMotorEx leftHex = null;
    public DcMotorEx rightHex = null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftHex = hardwareMap.get(DcMotorEx.class, "leftHex");
        rightHex = hardwareMap.get(DcMotorEx.class, "rightHex");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("left hex", leftHex.getCurrentPosition());
            telemetry.addData("right hex", rightHex.getCurrentPosition());
            telemetry.update();
        }
    }
}
