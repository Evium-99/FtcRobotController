package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Evium TeleOP")
public class Evium23 extends LinearOpMode {

    // Proportional, Integral, and Derivative Gains and Integral Summation for PID Controller
    // for the armature ran off of hex motors.
    public static double DISTANCE_FROM_BACKBOARD = 8.5;
    public static double armP = 0.00335;
    public static double armI = 0;
    public static double armD = 0.5;
    public static int backboardSetpoint = -230;
    public double integralSummation = 0;
    public double lastError = 0;

    // Declare OpMode members.

    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private DcMotorEx winchHex = null; // Winch Hex
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servo
    private Servo shoota = null; // Shooter Servo
     private DistanceSensor distanceBack = null; // Distance Sensor Back
    private DistanceSensor distanceFront = null; // Distance Sensor Front
    private TouchSensor armHit = null; // Arm Touch Sensor


    private boolean gamepad1AReleased = true; // Check if A is Released
    private boolean gamepad1XReleased = true; // Check if X is Released
    private boolean gamepadAReleased = true; // Check if A is Released
    private boolean gamepadXReleased = true; // Check is X is RELeased
    private boolean gamepad1BReleased = true; // Check if B is Released
    private boolean gamepadBReleased = true; // Check if B is Released
    private boolean gamepad1TriangleReleased = true;
    private boolean gamepad2Enabled = false;
    private boolean gamepad2Auto = false;
    ElapsedTime timer = new ElapsedTime();

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
        winchHex = hardwareMap.get(DcMotorEx.class, "rightEncoder");

        // Control Hub Servos
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        shoota = hardwareMap.get(Servo.class, "shooter");

        // Control Hub I2C
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        armHit = hardwareMap.get(TouchSensor.class, "armHit");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.REVERSE);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gripServo1.setPosition(1);
        gripServo2.setPosition(0);
        armServo.setPosition(0.13);

//        boolean stopArm = false;
        boolean gamepad1StartReleased = true;
        boolean sendingPower = false;
        int armCycle = 0;
//        int hexSetPoint = -7;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            if (leftHex.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
//                leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            }
            double y;
            double x;
            double rx;
            y = (-gamepad1.left_stick_y*0.25) + (Math.pow(-gamepad1.left_stick_y, 5)*0.75); // Remember, Y stick value is reversed
            x = (gamepad1.left_stick_x*0.25) + (Math.pow(gamepad1.left_stick_x, 5)*0.75); // Counteract imperfect strafing
            rx = (gamepad1.right_stick_x*0.25) + (Math.pow(gamepad1.right_stick_x, 5)*0.5);

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
            if (gamepad2.left_trigger > 0) {
                leftHex.setPower(-gamepad2.left_trigger);
                rightHex.setPower(-gamepad2.left_trigger);
                sendingPower = true;
            } else {
                leftHex.setPower(gamepad2.right_trigger);
                rightHex.setPower(gamepad2.right_trigger);
                sendingPower = true;
            }
//            if (armHit.isPressed() && !stopArm) {
//                rightHex.setMode(STOP_AND_RESET_ENCODER);
//                leftHex.setMode(STOP_AND_RESET_ENCODER);
//                stopArm = true;
//            } else if (armHit.isPressed() && stopArm) {
//                leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//                leftHex.setPower(-0.3);
//                rightHex.setPower(-0.3);
//            }
//
//            if (gamepad2.left_bumper || gamepad1.left_bumper) {
//                timer.reset();
//                hexSetPoint = -7;
//                integralSummation = 0;
//                lastError = 0;
//            }
//            if (gamepad2.right_bumper || gamepad1.right_bumper) {
//                armServo.setPosition(0.75);
//                timer.reset();
//                hexSetPoint = backboardSetpoint;
//                integralSummation = 0;
//                lastError = 0;
//            }

            // Grip Servo

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
                if (gripServo1.getPosition() != 1) {
                    gripServo1.setPosition(1);
                } else {
                    gripServo1.setPosition(0.4);
                }
            } else if (!gamepad2.a) {
                gamepad1AReleased = true;
            }

            if ((gamepad1XReleased) && (gamepad2.x)) {
                gamepad1XReleased = false;
                if (gripServo2.getPosition() != 1) {
                    gripServo2.setPosition(1);
                } else {
                    gripServo2.setPosition(-1);
                }
            } else if (!gamepad2.x) {
                gamepad1XReleased = true;
            }

//            double hexPower = pidController(hexSetPoint, leftHex.getCurrentPosition());
//            if (!armHit.isPressed()) {
//                stopArm = false;
//                if (hexPower < -1) {
//                    leftHex.setPower(-1);
//                    rightHex.setPower(-1);
//                } else if (hexPower > 1) {
//                    leftHex.setPower(1);
//                    rightHex.setPower(1);
//                } else {
//                    leftHex.setPower(hexPower);
//                    rightHex.setPower(hexPower);
//                }
//            }

            // Arm Servo
            if (armCycle == 0) {
                armServo.setPosition(0.13);
            } else if (armCycle == 1) {
                armServo.setPosition(0.75);
            }// gamepadBReleased
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
                    shoota.setPosition(1);
                } else if (shoota.getPosition() == 1) {
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
            // telemetry.addData("PID_Power", hexPower);
            telemetry.addData("HexLeft", leftHex.getPower());
            telemetry.addData("HexRight", rightHex.getPower());
            telemetry.addData("Sending Power", sendingPower);
//            telemetry.addData("Encoder Resetting", armHit.isPressed());
            telemetry.addData("Back Distance Sensor (in)", distanceBack.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
    public double pidController(double reference, double state) {
        double error = reference - state;
        telemetry.addData("Hex Error", error);
        integralSummation += error * timer.seconds();
        telemetry.addData("Hex Time", timer.seconds());
        double derivative = (error - lastError) / timer.seconds();
        telemetry.addData("Hex Derivative", derivative);
        lastError = error;
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
