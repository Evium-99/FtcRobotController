package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class servoTuner extends LinearOpMode {
    public static double grip1 = 0;
    public static double grip2 = 0;
    public static double arm = 0;
    public static double shootaVar = 0;

    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servo
    private Servo shoota = null; // Shooter Servo
    @Override
    public void runOpMode() throws InterruptedException {
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");
        shoota = hardwareMap.get(Servo.class, "shooter");
        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            gripServo1.setPosition(grip1);
            gripServo2.setPosition(grip2);
            armServo.setPosition(arm);
            shoota.setPosition(shootaVar);
            telemetry.addData("Time", timer.seconds());
            telemetry.addData("gripServo1", gripServo1.getPosition());
            telemetry.addData("gripServo2", gripServo2.getPosition());
            telemetry.addData("armServo", armServo.getPosition());
            telemetry.addData("shoota", shoota.getPosition());
            telemetry.update();
        }
    }
}
