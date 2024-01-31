package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

@Config
@Autonomous(name = "Blue Audiance v19")
public class autoBlueAudiance extends LinearOpMode {
    public static double DISTANCE_FROM_BACKBOARD = 3;
    public static double position = -270;
    public static double Z = 1;
    public static double distanceForward = 30;
    public static double backFromPixel = 5;
    public static double strafeRightB = 8 * Z;
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servo
    private DistanceSensor distanceBack = null; // Distance sensor on the back
    private DistanceSensor distanceFront = null; // Distance Sensor Front
    public String side = "Left";
    boolean toBoard = false;

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

        // Control Hub Servos
        gripServo2 = hardwareMap.get(Servo.class, "grip1");
        gripServo1 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");

        // Control Hub I2C
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        // Change The Left side to Backwards on Drive Motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.FORWARD);
        rightHex.setDirection(DcMotorEx.Direction.REVERSE);

        // Set Drive Constants
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set Hex Mode
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Pose estimate to 0, 0, 0 for initialization purposes
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Set Trajectory 1 to go forward
        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(startPose)
                .forward(distanceForward)
                .build();

        // Strafe Right 84
        TrajectorySequence ST84 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(84)
                .build();

        // Strafe Right 84
        TrajectorySequence ST96 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(106)
                .build();

        // Set turnAngleW to turn W
        TrajectorySequence turnAngleW = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-75))
                .build();

        // Set turnAngleW to turn W
        TrajectorySequence turnAngleG = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-115))
                .build();

        TrajectorySequence turnAngleP = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(70))
                .build();

        // Set turnAngleW to turn W
        TrajectorySequence turnAngleI = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-200))
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3b = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(strafeRightB)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3dba = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3dbb = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3bb = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(strafeRightB)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3c = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(5)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3cb = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .build();

        // Set 180Traj to go forward to board
        TrajectorySequence Traj180 = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(160))
                .build();

        // Set BackFromPixel to back backFromPixel
        TrajectorySequence BackFromPixel = drive.trajectorySequenceBuilder(startPose)
                .back(backFromPixel)
                .build();

        // Set BackFromPixel to back backFromPixel
        TrajectorySequence ForwardToPixel = drive.trajectorySequenceBuilder(startPose)
                .forward(backFromPixel)
                .build();

        // State initialization sequence is completed
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pos", leftHex.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        side = "Left";
        waitForStart();
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Squeeze Pixels
        gripServo1.setPosition(1);
        gripServo2.setPosition(0);

        // Update Telemetry
        telemetry.addData("Side", side);
        telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Pos", leftHex.getCurrentPosition());
        telemetry.update();

        // Move Away from Wall
        drive.followTrajectorySequence(trajectory1);

        // Wait for Robot to settle
        sleep(500);

        // If senses center, update telemetry and set side
        if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
            side = "Center";
            // Update Telemetry
            telemetry.addData("Side", side);
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("Pos", leftHex.getCurrentPosition());
            telemetry.update();
            drive.followTrajectorySequence(trajectory3b);
            sleep(700);
            armServo.setPosition(0);
            sleep(500);
            drive.followTrajectorySequence(trajectory3bb);
            sleep(700);
            gripServo2.setPosition(1);
        } else {
            // Turn Right
            drive.followTrajectorySequence(turnAngleW);

            // Wait for Robot to settle
            sleep(1000);

            // If senses right, update telemetry and set side
            if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
                side = "Right";
                telemetry.addData("Side", side);
                telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("Pos", leftHex.getCurrentPosition());
                telemetry.update();

                // Move to Right
                drive.followTrajectorySequence(trajectory3dba);
                sleep(700);

                // If Right drop Pixel
                armServo.setPosition(0);
                sleep(1100);
                drive.followTrajectorySequence(trajectory3dbb);
                sleep(700);
                gripServo2.setPosition(1);
                sleep(700);
                armServo.setPosition(0.94);
                sleep(500);
                drive.followTrajectorySequence(turnAngleP);
            } else {
                // If Left 180; Wait, Drop Pixel, wait; 180
                drive.followTrajectorySequence(Traj180); // 180
                armServo.setPosition(0); // Arm Down
                sleep(700);
                drive.followTrajectorySequence(trajectory3cb); // Right
                gripServo2.setPosition(1); // Drop Pixel
                sleep(700);
                drive.followTrajectorySequence(BackFromPixel); // Back
                sleep(700);
                armServo.setPosition(0.94);
                sleep(700);
                drive.followTrajectorySequence(turnAngleG);
            }
        }
        armServo.setPosition(0.94);
        while(!toBoard && opModeIsActive()){

            // Set Power to Motors Based on Log Func
            double power = -0.5 * ((0.6 * Math.log10(distanceBack.getDistance(DistanceUnit.INCH) - DISTANCE_FROM_BACKBOARD)) + 0.2);

            // Set Distance to Board Var for Easy Access
            double distance_to_board = distanceBack.getDistance(DistanceUnit.INCH);

            // Check if within tolerable distance to board
            if (distance_to_board < (DISTANCE_FROM_BACKBOARD + 0.4)) {
                // If tolerable, break out of loop
                toBoard = true;
                break;
            }

            if (power < 0) {
                motor1.setPower(power);
                motor2.setPower(power);
                motor3.setPower(power);
                motor4.setPower(power);
            }

            // Telemetry
            telemetry.addData("Distance (inch)", distance_to_board);
            telemetry.addData("Pos", rightHex.getCurrentPosition());
            telemetry.update();
        }
        if (Objects.equals(side, "Right")) {
            drive.followTrajectorySequence(ST96);
        } else {
            drive.followTrajectorySequence(ST84);
        }
        drive.followTrajectorySequence(turnAngleI);
        armServo.setPosition(0);
        sleep(1000);
        gripServo2.setPosition(0);
        sleep(1000);
        armServo.setPosition(0.5);
    }
}
