package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleOP 2W")
public class Evium2W extends LinearOpMode {
    private DcMotor motor0 = null; // Right
    private DcMotor motor3 = null; // Left

    public double pid(double iV, double oV, double kP) {
        double error = iV - oV;
        double value = error * kP;
        return value;
    }

    @Override
    public void runOpMode() {
        double val0 = 0;
        double val3 = 0;
        boolean aDown = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor0 = hardwareMap.get(DcMotor.class, "right");
        motor3 = hardwareMap.get(DcMotor.class, "left");
        motor0.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", gamepad1.right_stick_y, gamepad1.left_stick_y);
            telemetry.update();
            val0 = val0 + pid(gamepad1.right_stick_y, val0, 0.5);
            val3 = val3 + pid(gamepad1.left_stick_y, val3, 0.5);
            motor0.setPower(val0);
            motor3.setPower(val3);
            if (gamepad1.a) {
                if (!aDown) {
                    aDown = true;
                    motor0.setDirection(DcMotor.Direction.FORWARD);
                    motor3.setDirection(DcMotor.Direction.REVERSE);
                }
            } else {
                aDown = false;
            }
        }
    }
}
