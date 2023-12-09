package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;

@Config
@Autonomous(name="TEST_CompBRed", group = "Testing")
public class TestCompBRed extends LinearOpMode {
    public static int BACK = 28;
    public static int TURN = 100;
    public static int GO_TO_BOARD = 30;
    public static int ENCODER_POS = -368;
    public static int VELOCITY_1 = 220;
    public int VELOCITY_2 = 120;

    static final HashMap<String, Vector2d> positions = new HashMap<String, Vector2d>();

    static {
        positions.put("p1-1", new Vector2d(30.36, 43.72));
        positions.put("p1-2", new Vector2d(36, 43.72));
        positions.put("p1-3", new Vector2d(42.69, 43.72));
        positions.put("p1-4", new Vector2d(36, 12));
        positions.put("p1-5", new Vector2d(63.62, 44.91));

        positions.put("p2-1", new Vector2d(-41.83, 43.72));
        positions.put("p2-2", new Vector2d(-36, 43.72));
        positions.put("p2-3", new Vector2d(-29.42, 43.72));
        positions.put("p2-4", new Vector2d(-36, 12));
        positions.put("p2-5", new Vector2d(-61.61, 44.91));

        positions.put("p3-1", new Vector2d(-36, -60));
        positions.put("p3-2", new Vector2d(-24, -60));
        positions.put("p3-3", new Vector2d(-12, -60));
        positions.put("p3-4", new Vector2d(-36, -36));
        positions.put("p3-5", new Vector2d(-61.61, -44.16));

        positions.put("p4-1", new Vector2d(12, -60));
        positions.put("p4-2", new Vector2d(24, -60));
        positions.put("p4-3", new Vector2d(36, -60));
        positions.put("p4-4", new Vector2d(36, -36));
        positions.put("p4-5", new Vector2d(63.62, -45.06));

    }

    private WebcamName webcam1, webcam2;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // Declare OpMode members.
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotorEx leftHex = null; // Left Hex
    private DcMotorEx rightHex = null; // Right Hex
    private Servo armServo = null; // Arm Servo
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private DistanceSensor distanceRight = null; // Distance Sensor
    private DistanceSensor distanceLeft = null; // Distance Sensor
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        initAprilTag();

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
        leftHex = hardwareMap.get(DcMotorEx.class, "leftHex");
        rightHex = hardwareMap.get(DcMotorEx.class, "rightHex");

        // Control Hub Servos
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
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
        leftHex.setDirection(DcMotorEx.Direction.FORWARD);
        rightHex.setDirection(DcMotorEx.Direction.FORWARD);
        leftHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(-60, -40, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                .forward(BACK)
                .turn(Math.toRadians(TURN))
                .back(GO_TO_BOARD) // Originally 34
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gripServo2.setPosition(-1); // Close Purple Side
        gripServo1.setPosition(1); // Close Yellow Side?
        armServo.setPosition(0.8);
        drive.followTrajectorySequence(ts);
        armServo.setPosition(0);
        leftHex.setTargetPosition(ENCODER_POS);
        rightHex.setTargetPosition(ENCODER_POS);
        leftHex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightHex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rightHex.setVelocity(VELOCITY_1);
        leftHex.setVelocity(VELOCITY_1);

        // run until the end of the match (driver presses STOP)
        double t = getRuntime();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (leftHex.getCurrentPosition() < -250) {
                rightHex.setVelocity(VELOCITY_2);
                leftHex.setVelocity(VELOCITY_2);
            }
            if ((getRuntime() - t) > 1) {
                armServo.setPosition(1);
            }
            if (((getRuntime() - t) > 4) && !((getRuntime() - t) > 5)) {
                gripServo2.setPosition(1); // Open Purple Side
                gripServo1.setPosition(-1); // Open Yellow Side?
            }
            if ((getRuntime() - t) > 5.5) {
                leftHex.setTargetPosition(-20);
                rightHex.setTargetPosition(-20);
                leftHex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightHex.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightHex.setVelocity(200);
                leftHex.setVelocity(200);
            }
            if ((getRuntime() - t) > 9) {
                gripServo2.setPosition(1); // Open Purple Side
                gripServo1.setPosition(-1); // Open Yellow Side?
            }
            telemetry.addData("HexPos", leftHex.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }
}