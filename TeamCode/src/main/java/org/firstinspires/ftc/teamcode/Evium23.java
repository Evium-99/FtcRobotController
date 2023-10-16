package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Evium TeleOP 2")
public class Evium23 extends LinearOpMode {

    // Declare OpMode members.

    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private Servo gripServo = null; // Grip Servo
    private Servo armServo = null; // Arm Servo

    private boolean gamepad1AReleased = true; // Check if A is Released
    private boolean gamepad1BReleased = true; // Check if B is Released

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Control Hub Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");

        // Control Hub Servos
        gripServo = hardwareMap.get(Servo.class, "grip");
        armServo = hardwareMap.get(Servo.class, "arm");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotor.Direction.FORWARD);
        rightHex.setDirection(DcMotor.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motor1.setPower(frontRightPower);
            motor2.setPower(frontLeftPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

            // Up and Down Arm Base
            if ((gamepad1.left_trigger != 0)) {
                leftHex.setPower(gamepad1.left_trigger);
                rightHex.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger != 0) {
                leftHex.setPower(-gamepad1.right_trigger);
                rightHex.setPower(-gamepad1.right_trigger);
            } else {
                leftHex.setPower(0);
                rightHex.setPower(0);
            }

            // Grip Servo
            if ((gamepad1AReleased) && (gamepad1.a)) {
                gamepad1AReleased = false;
                if (gripServo.getPosition() == 0) {
                    gripServo.setPosition(1);
                } else if (gripServo.getPosition() == 1) {
                    gripServo.setPosition(0);
                } else {
                    gripServo.setPosition(0);
                }
            } else if (!gamepad1.a) {
                gamepad1AReleased = true;
            }

            // Arm Servo
            if ((gamepad1BReleased) && (gamepad1.b)) {
                gamepad1BReleased = false;
                if (armServo.getPosition() == 0) {
                    armServo.setPosition(0.9);
                } else if (armServo.getPosition() == 1) {
                    armServo.setPosition(0);
                } else {
                    armServo.setPosition(0);
                }
            } else if (!gamepad1.b) {
                gamepad1BReleased = true;
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Left Hex Motor Value", leftHex.getCurrentPosition());
            telemetry.addData("Right Hex Motor Value", rightHex.getCurrentPosition());
            telemetry.update();
        }
    }
}
