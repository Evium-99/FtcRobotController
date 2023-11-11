package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Arm Auto")
public class ArmTuner extends LinearOpMode {
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private TouchSensor armHit = null; // Arm Touch Sensor

    @Override
    public void runOpMode() {
        // Down -20
        // Back -377
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");
        armHit = hardwareMap.get(TouchSensor.class, "armHit");
        leftHex.setDirection(DcMotor.Direction.FORWARD);
        rightHex.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized v2");
        telemetry.update();
        waitForStart();

        boolean stopArm = false;
        double downPower = 0.5;

        while (opModeIsActive()) {
            if (armHit.isPressed() && !stopArm) {
                rightHex.setMode(STOP_AND_RESET_ENCODER);
                leftHex.setMode(STOP_AND_RESET_ENCODER);
                stopArm = true;
            } else if (armHit.isPressed() && stopArm) {
                leftHex.setTargetPosition(-20);
                rightHex.setTargetPosition(-20);
                leftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHex.setPower(downPower);
                rightHex.setPower(downPower);
            }
            if (!armHit.isPressed()) {
                stopArm = false;
            }
            if (gamepad1.a) {
                leftHex.setTargetPosition(-20);
                rightHex.setTargetPosition(-20);
                leftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHex.setPower(downPower);
                rightHex.setPower(downPower);
            }
            if (gamepad1.b) {
                leftHex.setTargetPosition(-377);
                rightHex.setTargetPosition(-377);
                leftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHex.setPower(downPower);
                rightHex.setPower(downPower);
            }
            telemetry.addData("Left", leftHex.getCurrentPosition());
            telemetry.addData("Right", rightHex.getCurrentPosition());
            telemetry.update();
        }
    }
}
