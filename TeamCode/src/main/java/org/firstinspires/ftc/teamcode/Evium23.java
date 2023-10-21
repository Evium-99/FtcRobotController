package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private DistanceSensor distanceRight = null; // Distance Sensor
    private DistanceSensor distanceLeft = null; // Distance Sensor

    private boolean gamepad1AReleased = true; // Check if A is Released
    private boolean gamepad1BReleased = true; // Check if B is Released
    private boolean gamepad1TriangleReleased = true;
    private boolean gamepad2Enabled = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double dPadTimeout = runtime.milliseconds();

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

        // Control Hub I2C
        // distanceLeft = hardwareMap.get(DistanceSensor.class, "BLDS");
        // distanceRight = hardwareMap.get(DistanceSensor.class, "BRDS");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotor.Direction.FORWARD);
        rightHex.setDirection(DcMotor.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gripServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y;
            double x;
            double rx;
            if (gamepad2Enabled) {
                y = Math.pow(-gamepad2.left_stick_y, 3); // Remember, Y stick value is reversed
                x = Math.pow(gamepad2.left_stick_x * 1.1, 3); // Counteract imperfect strafing
                rx = gamepad2.right_stick_x;
            } else {
                y = Math.pow(-gamepad1.left_stick_y, 3); // Remember, Y stick value is reversed
                x = Math.pow(gamepad1.left_stick_x * 1.1, 3); // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.x) {
                motor1.setPower(0.5);
                motor2.setPower(0.5);
                motor3.setPower(0.5);
                motor4.setPower(0.5);
            } else {
                motor1.setPower(frontRightPower);
                motor2.setPower(frontLeftPower);
                motor3.setPower(backLeftPower);
                motor4.setPower(backRightPower);
            }

            // Up and Down Arm Base
            if ((gamepad1.left_trigger != 0)) {
                leftHex.setPower(0.5*Math.pow(gamepad1.left_trigger, 3));
                rightHex.setPower(0.5*Math.pow(gamepad1.left_trigger, 3));
            } else if (gamepad1.right_trigger != 0) {
                leftHex.setPower(0.7*Math.pow(-gamepad1.right_trigger, 3));
                rightHex.setPower(0.7*Math.pow(-gamepad1.right_trigger, 3));
            } else {
                leftHex.setPower(0);
                rightHex.setPower(0);
            }

            // Second Controller Transfer
            if (gamepad1TriangleReleased && gamepad1.y) {
                gamepad1TriangleReleased = false;
                gamepad2Enabled = !gamepad2Enabled;
            } else if (!gamepad1.y) {
                gamepad1TriangleReleased = true;
            }

            // Grip Servo

            if ((gamepad1AReleased) && (gamepad1.a)) {
                gamepad1AReleased = false;
                if (gripServo.getPosition() == 0.5) {
                    gripServo.setPosition(1);
                } else if (gripServo.getPosition() == 1) {
                    gripServo.setPosition(0.5);
                } else {
                    gripServo.setPosition(0.5);
                }
            } else if (!gamepad1.a) {
                gamepad1AReleased = true;
            }

            // Arm Servo Manual Control armServo.setPosition(0)
            if (gamepad1.dpad_up  && (runtime.milliseconds() > (dPadTimeout + 100))) {
                dPadTimeout = runtime.milliseconds();
                armServo.setPosition(armServo.getPosition()+0.03);
            } else if (gamepad1.dpad_down) {
                dPadTimeout = runtime.milliseconds();
                armServo.setPosition(armServo.getPosition()-0.03);
            }

            // Arm Servo
            if ((gamepad1BReleased) && (gamepad1.b)) {
                gamepad1BReleased = false;
                if (armServo.getPosition() == 0) {
                    armServo.setPosition(1);
                } else if (armServo.getPosition() == 1) {
                    armServo.setPosition(0);
                } else {
                    armServo.setPosition(0);
                }
            } else if (!gamepad1.b) {
                gamepad1BReleased = true;
            }

            // Show the elapsed game time and wheel power.
            // telemetry.addData("Distance Sensor Left", distanceLeft.getDistance(DistanceUnit.CM));
            // telemetry.addData("Distance Sensor Right", distanceRight.getDistance(DistanceUnit.CM));
            // telemetry.addData("Left Hex Motor Value", leftHex.getCurrentPosition());
            // telemetry.addData("Right Hex Motor Value", rightHex.getCurrentPosition());
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.update();
        }
    }
}
