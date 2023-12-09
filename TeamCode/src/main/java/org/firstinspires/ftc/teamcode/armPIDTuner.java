package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name="ArmPIDTuner")
public class armPIDTuner extends LinearOpMode {
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
    private Servo armServo = null; // Arm Servo
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Not Ready");
        telemetry.update();

        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotorEx.class, "leftHex");
        rightHex = hardwareMap.get(DcMotorEx.class, "rightHex");
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setDirection(DcMotorEx.Direction.REVERSE);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
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
            telemetry.addData("Status", "Main Loop");
            telemetry.update();
            double power = pidController(position, leftHex.getCurrentPosition());
            leftHex.setPower(power);
            rightHex.setPower(power);
            telemetry.addData("Current Pos", leftHex.getCurrentPosition());
            telemetry.update();
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
