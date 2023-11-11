package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="Evium TeleOP")
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
    private Servo shoota = null; // Shooter Servo
    private DistanceSensor distanceRight = null; // Distance Sensor
    private DistanceSensor distanceLeft = null; // Distance Sensor
    private TouchSensor armHit = null; // Arm Touch Sensor

    private boolean gamepad1AReleased = true; // Check if A is Released
    private boolean gamepad1BReleased = true; // Check if B is Released
    private boolean gamepad1TriangleReleased = true;
    private boolean gamepad2Enabled = false;

    @Override
    public void runOpMode() {
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
        shoota = hardwareMap.get(Servo.class, "shooter");

        // Control Hub I2C
        // distanceLeft = hardwareMap.get(DistanceSensor.class, "BLDS");
        // distanceRight = hardwareMap.get(DistanceSensor.class, "BRDS");
        armHit = hardwareMap.get(TouchSensor.class, "armHit");

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

        boolean stopArm = false;
        double downPower = 0.5;
        boolean gamepad1StartReleased = true;

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

            motor1.setPower(frontRightPower);
            motor2.setPower(frontLeftPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

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
            if (gamepad1.left_bumper) {
                leftHex.setTargetPosition(-20);
                rightHex.setTargetPosition(-20);
                leftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHex.setPower(downPower);
                rightHex.setPower(downPower);
            }
            if (gamepad1.right_bumper) {
                leftHex.setTargetPosition(-377);
                rightHex.setTargetPosition(-377);
                leftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftHex.setPower(downPower);
                rightHex.setPower(downPower);
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
            // shoota servo
            if ((gamepad1StartReleased) && (gamepad1.start)) {
                gamepad1StartReleased = false;
                if (shoota.getPosition() == 0) {
                    shoota.setPosition(1);
                } else if (shoota.getPosition() == 1) {
                    shoota.setPosition(0);
                } else {
                    shoota.setPosition(0);
                }
            } else if (!gamepad1.start) {
                gamepad1StartReleased = true;
            }
        }
    }
}
