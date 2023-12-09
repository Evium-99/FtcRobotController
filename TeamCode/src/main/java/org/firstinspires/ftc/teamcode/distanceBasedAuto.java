package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Distance Based Auto")
public class distanceBasedAuto extends LinearOpMode {
    public static double angle = 90;
    public static double distanceForward = 28;
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
    private DistanceSensor distanceFront = null; // Distance Sensor Front
    private DistanceSensor distanceFrontLeft = null; // Distance Sensor Front Left
    private TouchSensor armHit = null; // Arm Touch Sensor
    public String side = "Left";

    @Override
    public void runOpMode() throws InterruptedException {
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
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceFrontLeft = hardwareMap.get(DistanceSensor.class, "frontLeft");
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

        // Set Drive Constants
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Pose estimate to 0, 0, 0 for initialization purposes
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Set Trajectory 1 to go forward
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(distanceForward)
                .build();

        // Set Trajectory 2 to turn right
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(startPose)
                .turn(-Math.toRadians(angle))
                .build();

        // State initialization sequence is completed
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        side = "Left";
        waitForStart();

        // Update Telemetry
        telemetry.addData("Side", side);
        telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
        telemetry.update();

        // Move Away from Wall
        drive.followTrajectorySequence(trajectory1);

        // Wait for Robot to settle
        sleep(1000);

        // If senses center, update telemetry and set side
        if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
            side = "Center";
            // Update Telemetry
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        // Turn Right
        drive.followTrajectorySequence(trajectory2);

        // Wait for Robot to settle
        sleep(1000);

        // If senses right, update telemetry and set side
        if (distanceFrontLeft.getDistance(DistanceUnit.INCH) < 10) {
            side = "Right";
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

        // Read Data to Screen
        while (opModeIsActive()) {
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
