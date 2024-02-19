package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp(name="Evium TeleOP")
public class Evium23 extends LinearOpMode {

    // Proportional, Integral, and Derivative Gains and Integral Summation for PID Controller
    // for the armature ran off of hex motor.
    public static double DISTANCE_FROM_BACKBOARD = 11;

    private boolean gamepad1AReleased = true; // Check if A is Released
    private boolean gamepad1XReleased = true; // Check if X is Released
    private boolean gamepadAReleased = true; // Check if A is Released
    private boolean gamepadXReleased = true; // Check is X is RELeased
    private boolean gamepad1BReleased = true; // Check if B is Released
    private boolean gamepadBReleased = true; // Check if B is Released

    @Override
    public void runOpMode() {
        // Send Telemetry to FtcDashboard
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Control Hub Motors
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        DcMotorEx motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        DcMotorEx motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        // Expansion Hub Motors
        DcMotorEx leftHex = hardwareMap.get(DcMotorEx.class, "leftHex");
        DcMotorEx rightHex = hardwareMap.get(DcMotorEx.class, "rightHex");
        DcMotorEx winchHex = hardwareMap.get(DcMotorEx.class, "rightEncoder");

        // Control Hub Servos
        Servo gripServo1 = hardwareMap.get(Servo.class, "grip1");
        Servo gripServo2 = hardwareMap.get(Servo.class, "grip2");
        Servo wristServo = hardwareMap.get(Servo.class, "arm");
        Servo shoota = hardwareMap.get(Servo.class, "shooter");

        // Control Hub I2C
        DistanceSensor distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.REVERSE);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        boolean gamepad1StartReleased = true;
        int armCycle = 0;
        double servoOffset = 0;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Auto Close
        gripServo1.setPosition(0);
        gripServo2.setPosition(1);
        shoota.setPosition(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double y;
            double x;
            double rx;
            if (gamepad1.left_bumper) {
                y = 0.5 * ((-gamepad1.left_stick_y*0.8) + (Math.pow(-gamepad1.left_stick_y, 3)*0.2)); // Remember, Y stick value is reversed
                x = 0.5 * ((gamepad1.left_stick_x*0.8) + (Math.pow(gamepad1.left_stick_x, 3)*0.2)); // Counteract imperfect strafing
                rx = 0.5 * ((gamepad1.right_stick_x*0.8) + (Math.pow(gamepad1.right_stick_x, 3)*0.2));
            } else {
                y = (-gamepad1.left_stick_y*0.5) + (Math.pow(-gamepad1.left_stick_y, 9)*0.5); // Remember, Y stick value is reversed
                x = (gamepad1.left_stick_x*0.5) + (Math.pow(gamepad1.left_stick_x, 9)*0.5); // Counteract imperfect strafing
                rx = (gamepad1.right_stick_x*0.5) + (Math.pow(gamepad1.right_stick_x, 9)*0.5);
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

            if (gamepad1.left_trigger == 0) {
                winchHex.setPower(gamepad1.right_trigger);
            } else {
                winchHex.setPower(-gamepad1.left_trigger);
            }

            // Auto Control
            if (gamepad1.y || gamepad2.y) {
                gripServo1.setPosition(0);
                gripServo2.setPosition(1);
            }
            double speedThresh2 = 0.90;
            if ((y > speedThresh2) || (y < -speedThresh2) || (x > speedThresh2) || (x < -speedThresh2) || (rx > speedThresh2) || (rx < -speedThresh2)) {
                gripServo1.setPosition(0);
                gripServo2.setPosition(1);
            }
            if (gamepad2.left_trigger > 0) {
                leftHex.setPower(-gamepad2.left_trigger);
                rightHex.setPower(-gamepad2.left_trigger);
            } else {
                leftHex.setPower(gamepad2.right_trigger);
                rightHex.setPower(gamepad2.right_trigger);
            }

            if ((gamepadAReleased) && (gamepad1.a)) {
                gamepadAReleased = false;
                if (gripServo1.getPosition() != 1) {
                    gripServo1.setPosition(1);
                } else {
                    gripServo1.setPosition(0.4);
                }
            } else if (!gamepad1.a) {
                gamepadAReleased = true;
            }

            if ((gamepadXReleased) && (gamepad1.x)) {
                gamepadXReleased = false;
                if (gripServo2.getPosition() != 1) {
                    gripServo2.setPosition(1);
                } else {
                    gripServo2.setPosition(-1);
                }
            } else if (!gamepad1.x) {
                gamepadXReleased = true;
            }

            if ((gamepad1AReleased) && (gamepad2.a)) {
                gamepad1AReleased = false;
                if (gripServo2.getPosition() != 1) {
                    gripServo2.setPosition(1);
                } else {
                    gripServo2.setPosition(0.4);
                }
            } else if (!gamepad2.a) {
                gamepad1AReleased = true;
            }

            if ((gamepad1XReleased) && (gamepad2.x)) {
                gamepad1XReleased = false;
                if (gripServo1.getPosition() != 1) {
                    gripServo1.setPosition(1);
                } else {
                    gripServo1.setPosition(-1);
                }
            } else if (!gamepad2.x) {
                gamepad1XReleased = true;
            }

            // Arm Servo
            if (gamepad1.back) {
                if (gamepad1.dpad_up) {
                    servoOffset = servoOffset + 0.01;
                } else if (gamepad1.dpad_down) {
                    servoOffset = servoOffset - 0.01;
                }
            } else {
                if (gamepad1.dpad_up) {
                    servoOffset = servoOffset + 0.001;
                } else if (gamepad1.dpad_down) {
                    servoOffset = servoOffset - 0.001;
                }
            }
            // Drive Threshold
            double speedThrech = 0.3;

            if (armCycle == 0) {
                servoOffset = 0;
                if (!gamepad1.left_bumper && ((y > speedThrech) || (y < -speedThrech) || (x > speedThrech) || (x < -speedThrech) || (rx > speedThrech) || (rx < -speedThrech))) {
                    wristServo.setPosition(0.2);
                } else {
                    wristServo.setPosition(0);
                }
            } else {
                wristServo.setPosition(0.75 + servoOffset);
            }
            // gamepadBReleased
            if ((gamepadBReleased) && (gamepad1.b)) {
                gamepadBReleased = false;
                if (armCycle == 0) {
                    armCycle = 1;
                } else {
                    armCycle = 0;
                }
            } else if (!gamepad1.b) {
                gamepadBReleased = true;
            }

            if ((gamepad1BReleased) && (gamepad2.b)) {
                gamepad1BReleased = false;
                if (armCycle == 0) {
                    armCycle = 1;
                } else {
                    armCycle = 0;
                }
            } else if (!gamepad2.b) {
                gamepad1BReleased = true;
            }
            // shoota servo
            if ((gamepad1StartReleased) && (gamepad2.start)) {
                gamepad1StartReleased = false;
                if (shoota.getPosition() == 0) {
                    shoota.setPosition(0.5);
                } else if (shoota.getPosition() == 0.5) {
                    shoota.setPosition(0);
                } else {
                    shoota.setPosition(0);
                }
            } else if (!gamepad2.start) {
                gamepad1StartReleased = true;
            }

            // Move back
            if (gamepad1.y) {
                double power = -((0.6*Math.log10(distanceBack.getDistance(DistanceUnit.INCH)-DISTANCE_FROM_BACKBOARD))+0.2);
                if (distanceBack.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_BACKBOARD) {
                    motor1.setPower(0.1);
                    motor2.setPower(0.1);
                    motor3.setPower(0.1);
                    motor4.setPower(0.1);
                } else if (power < 0) {
                    motor1.setPower(power);
                    motor2.setPower(power);
                    motor3.setPower(power);
                    motor4.setPower(power);
                }
            }
            telemetry.addData("Arm Pos", leftHex.getCurrentPosition());
            telemetry.addData("Gamepad1 Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Gamepad1 Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad1 Right Stick X", gamepad1.right_stick_x);

            telemetry.addData("Back Distance Sensor (in)", distanceBack.getDistance(DistanceUnit.INCH));
            if (gripServo1.getPosition() != 1) {
                telemetry.addData("Left Grip (A)", "Close");
            } else {
                telemetry.addData("Left Grip (A)", "Open");
            }
            if (gripServo2.getPosition() != 1) {
                telemetry.addData("Right Grip (X)", "Open");
            } else {
                telemetry.addData("Right Grip (X)", "Close");
            }
            if (armCycle == 0) {
                telemetry.addData("Wrist (B)", "Down");
            } else {
                telemetry.addData("Wrist (B)", "Up");
            }
            telemetry.update();
        }
    }
}
