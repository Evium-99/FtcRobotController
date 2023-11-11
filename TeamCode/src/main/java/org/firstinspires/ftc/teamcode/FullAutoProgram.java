package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.HashMap;
import java.util.List;

@Autonomous(name="FullAutoProgram")
public class FullAutoProgram extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Red & Blue Prop.tflite";
    private static final String[] LABELS = {
            "Blue Prop",
            "Red Prop"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    static final HashMap<Integer, Pose2d> aprils = new HashMap<Integer, Pose2d>();
    static {
        aprils.put(1, new Pose2d(-42.83, 63.5, Math.toRadians(90)));
        aprils.put(2, new Pose2d(-36.83, 63.5, Math.toRadians(90)));
        aprils.put(3, new Pose2d(-30.83, 63.5, Math.toRadians(90)));
        aprils.put(4, new Pose2d(30.83, 63.5, Math.toRadians(90)));
        aprils.put(5, new Pose2d(36.83, 63.5, Math.toRadians(90)));
        aprils.put(6, new Pose2d(42.83, 63.5, Math.toRadians(90)));
        aprils.put(7, new Pose2d(43, -72, Math.toRadians(270)));
        aprils.put(8, new Pose2d(37.5, -72, Math.toRadians(270)));
        aprils.put(9, new Pose2d(-37.5, -72, Math.toRadians(270)));
        aprils.put(10, new Pose2d(-43, -72, Math.toRadians(270)));
    }

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
    private AprilTagProcessor aprilTag;

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

    private Pose2d startPose;

    @Override
    public void runOpMode() {

        initVision();

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
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startPose = localize();
        if (startPose == null) {
            startPose = new Pose2d(-60, -40, Math.toRadians(90));
        }

        drive.setPoseEstimate(startPose);

        TrajectorySequence lookLeft = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(30)) // Turns 15 degrees counter-clockwise
                .build();

        TrajectorySequence lookRight = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-30)) // Turns 15 degrees clockwise
                .build();

        TrajectorySequence moveUpABit = drive.trajectorySequenceBuilder(startPose)
                .forward(5)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Send Power to Grip Pixel
        gripServo.setPosition(1);
        armServo.setPosition(0.5);
        // Look For Team Prop by looking left and right
        telemetryTfod();
        telemetry.update();
        drive.followTrajectorySequence(moveUpABit); // Move Up a Bit
        telemetryTfod();
        telemetry.update();
        drive.followTrajectorySequence(lookLeft); // Looking Left
        telemetryTfod();
        telemetry.update();
        drive.followTrajectorySequence(lookRight); // Back to Center
        telemetryTfod();
        telemetry.update();
        drive.followTrajectorySequence(lookRight); // Looking Right
        telemetryTfod();
        telemetry.update();
        // Drop Pixel
        armServo.setPosition(1);
        gripServo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetryTfod();
            telemetry.update();
        }
        visionPortal.close();
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }

    private void initVision() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);

        // Create Processor for April Tags
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Set and enable the processor for tfod and apriltags.
        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.72f);
    }

    private Pose2d aprilToBotLocation(int tagID, double botRelationX, double botRelationY, double yaw) {
        Pose2d tag = aprils.get(tagID);
        if(tag == null){
            return startPose;
        }
        return new Pose2d(tag.getX() - botRelationX, tag.getY() - botRelationY, tag.getHeading() - yaw);
    }

    private Pose2d localize() {
        Pose2d latestPose = null;
        ElapsedTime localizationRuntime = new ElapsedTime();
        localizationRuntime.reset();
        localizationRuntime.startTime();
        while (localizationRuntime.milliseconds()/1000 < 3) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) { // detection.ftcPose.x, detection.ftcPose.y,
                    latestPose = aprilToBotLocation(detection.id, detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
                }
            }
        }
        localizationRuntime.reset();
        return latestPose;
    }

}
