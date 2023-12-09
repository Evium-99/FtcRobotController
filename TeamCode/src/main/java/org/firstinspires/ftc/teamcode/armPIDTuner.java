package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name="ArmPIDTuner")
public class armPIDTuner extends LinearOpMode {
    public static double DISTANCE_FROM_BACKBOARD = 8.7;
    public static double armP = 0.004;
    public static double armI = 0;
    public static double armD = 0.4;
    public double integralSummation = 0;
    public double lastError = 0;
    public static double position = -300;


    private DcMotorEx leftHex = null; // Left Hex
    private DcMotorEx rightHex = null; // Right Hex
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servoprivate DcMotor motor1 = null; // Front Right
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right

    private DistanceSensor distanceBack = null;
    ElapsedTime timer = new ElapsedTime();
    boolean stopPower = false;
    boolean toBoard = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Not Ready");
        telemetry.update();

        // Hardware Mapping
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        leftHex = hardwareMap.get(DcMotorEx.class, "leftHex");
        rightHex = hardwareMap.get(DcMotorEx.class, "rightHex");
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setDirection(DcMotorEx.Direction.REVERSE);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", "Ready");
        telemetry.addData("Current Pos", leftHex.getCurrentPosition());
        telemetry.update();
        waitForStart();
        telemetry.addData("Status", "Running");
        telemetry.addData("Current Pos", leftHex.getCurrentPosition());
        telemetry.update();
        armServo.setPosition(0.75);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gripServo1.setPosition(1);
        gripServo2.setPosition(0);

        while (opModeIsActive()) {
            if (!toBoard) {
                double power = -0.5*((0.6*Math.log10(distanceBack.getDistance(DistanceUnit.INCH)-DISTANCE_FROM_BACKBOARD))+0.2);
                if ((distanceBack.getDistance(DistanceUnit.INCH) > (DISTANCE_FROM_BACKBOARD-0.4)) && (distanceBack.getDistance(DistanceUnit.INCH) < (DISTANCE_FROM_BACKBOARD+0.4))) {
                    toBoard = true;
                } else if (distanceBack.getDistance(DistanceUnit.INCH) < DISTANCE_FROM_BACKBOARD) {
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
                telemetry.addData("Distance (inch)", distanceBack.getDistance(DistanceUnit.INCH));
                telemetry.update();
            } else {
                motor1.setPower(0);
                motor2.setPower(0);
                motor3.setPower(0);
                motor4.setPower(0);
                if (!stopPower) {
                    telemetry.addData("Status", "Main Loop");
                    telemetry.update();
                    double power = pidController(position, leftHex.getCurrentPosition());
                    if (((position + 10) > leftHex.getCurrentPosition()) && (leftHex.getCurrentPosition() > position - 10)) {
                        stopPower = true;
                    }
                    leftHex.setPower(power);
                    rightHex.setPower(power);
                } else {
                    leftHex.setPower(0);
                    rightHex.setPower(0);
                }
                telemetry.addData("Current Pos", leftHex.getCurrentPosition());
                telemetry.update();
            }
        }
    }
    public double pidController(double reference, double state) {
        double error = reference - state;
        integralSummation += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
