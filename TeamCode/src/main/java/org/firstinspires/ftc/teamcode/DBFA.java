package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "DBFullAuto")
public class DBFA extends LinearOpMode {
    // Declare OpMode members.
    private Servo gripServo1; // Grip Servo 1
    private Servo gripServo2; // Grip Servo 2
    private DcMotor motor1; // Front Right
    private DcMotor motor2; // Front Left
    private DcMotor motor3; // Back Left
    private DcMotor motor4; // Back Right
    private DcMotor leftHex; // Left Hex
    private DcMotor rightHex; // Right Hex
    private Servo armServo; // Arm Servo
    private DistanceSensor distanceBack; // Distance Sensor Back
    private DistanceSensor distanceFront; // Distance Sensor Front
    private TouchSensor armHit; // Arm Touch Sensor
    private ElapsedTime runtime = new ElapsedTime();

    private Pose2d startPose;
    @Override
    public void runOpMode() throws InterruptedException {
        // Control Hub I2C
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        armHit = hardwareMap.get(TouchSensor.class, "armHit");
        
        // Control Hub Motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotor.Direction.FORWARD);
        rightHex.setDirection(DcMotor.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.REVERSE);


        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");

        // Control Hub Servos
        armServo = hardwareMap.get(Servo.class, "arm");
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive.setPoseEstimate(startPose);

        TrajectorySequence moveUpABit = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    telemetry.addData("Status", "Move From Wall");
                    telemetry.update();
                    // Send Power to Grip Pixel
                    gripServo1.setPosition(1);
                    gripServo2.setPosition(-1);
                    armServo.setPosition(1);
                })
                .forward(12)
                .build();
    }
}
