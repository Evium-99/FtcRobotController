package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;

@Autonomous(name="Competition Autonomous")
public class CompAuto extends LinearOpMode {

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
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private Servo gripServo = null; // Grip Servo
    private Servo armServo = null; // Arm Servo
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-60, -40, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d(-60, -40, Math.toRadians(90)))
                .splineTo(positions.get("p3-4"), Math.toRadians(90))
                .splineTo(positions.get("p2-4"), Math.toRadians(90))
                .splineTo(positions.get("p2-1"), Math.toRadians(90))
                .build();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        drive.followTrajectory(myTrajectory);
        gripServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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

    private void localize() {
        ElapsedTime localizationRuntime = new ElapsedTime();
        localizationRuntime.reset();
        localizationRuntime.startTime();
        while (localizationRuntime.milliseconds()/1000 > 3) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {} else {}
            }
        }
        localizationRuntime.reset();
    }

}
