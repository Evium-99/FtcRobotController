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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue Backdrop v52")
public class autoBlueBackdrop extends LinearOpMode {
    public static double DISTANCE_FROM_BACKBOARD = 10;
    public static double armP = 0.0025;
    public static double armI = 0;
    public static double armD = 0.2;
    public double integralSummation = 0;
    public double lastError = 0;
    public static double position = -270;
    public static double positionb = -10;
    public static double W = 1;
    public static double Z = -1;
    public static double Public_Angle = 90;
    public static double CenterOffsetDeg = 12 * W;
    public static double rightOffset = 15 * Z;
    public static double rightOffsetDeg = 30 * Z;
    public static double aSmidgeLeft = 10;
    private static final double angleZ = Public_Angle * Z;
    public static double distanceForward = 30;
    public static double backFromPixel = 5;
    public static double backFromPixelXS = 2.5;
    public static double strafeRightG = 10 * W;
    public static double strafeRightC = 24 * W;
    public static double strafeLeftC = 16 * W;
    public static double toBoardFromCenter = 30;
    // Back 13
    public static double X_BBS_FORWARDa = 22;
    // Strafe Left 15
    private DcMotor leftHex = null; // Left Hex
    private DcMotor rightHex = null; // Right Hex
    private Servo gripServo1 = null; // Grip Servo 1
    private Servo gripServo2 = null; // Grip Servo 2
    private Servo armServo = null; // Arm Servo
    private DistanceSensor distanceBack = null; // Distance sensor on the back
    private DistanceSensor distanceFront = null; // Distance Sensor Front
    public String side = "Left";
    ElapsedTime timer = new ElapsedTime();
    boolean stopPower = false;
    boolean stopPowerb = false;
    boolean toBoard = false;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        // Control Hub Motors
        // Front Right
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor motor3 = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Expansion Hub Motors
        leftHex = hardwareMap.get(DcMotor.class, "leftHex");
        rightHex = hardwareMap.get(DcMotor.class, "rightHex");

        // Control Hub Servos
        gripServo1 = hardwareMap.get(Servo.class, "grip1");
        gripServo2 = hardwareMap.get(Servo.class, "grip2");
        armServo = hardwareMap.get(Servo.class, "arm");

        // Control Hub I2C
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");

        // Set Core Hex Motor Direction, Mode, and Breaking
        leftHex.setDirection(DcMotorEx.Direction.FORWARD);
        rightHex.setDirection(DcMotorEx.Direction.REVERSE);
        leftHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightHex.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        // Set turnAngleZ to turn Z
        TrajectorySequence turnAngleZ = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(angleZ))
                .build();

        // Set turnAngleZ to turn Z
        TrajectorySequence turnAngleP = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-70))
                .build();

        // Set Trajectory Strafe Right
        TrajectorySequence StrafeRightG = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(2, -strafeRightG))
                .build();

        // Set Trajectory Strafe Right
        TrajectorySequence StrafeRightO = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(-strafeRightG)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3b = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(10)
                .build();

        // Set Trajectory 3 to strafe right
        TrajectorySequence trajectory3bb = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(10)
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3c = drive.trajectorySequenceBuilder(startPose) // X= forward Back
                .strafeTo(new Vector2d(-24, 9))
                .build();

        // Set Trajectory 3c to strafe right
        TrajectorySequence trajectory3cb = drive.trajectorySequenceBuilder(startPose) // X= forward Back
                .turn(Math.toRadians(-40))
                .build();

        // Set toBoardFromCenterTraj to go back to board
        TrajectorySequence toBoardFromCenterTraj = drive.trajectorySequenceBuilder(startPose)
                .back(toBoardFromCenter)
                .build();

        TrajectorySequence BackFromPixelMega = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .build();

        TrajectorySequence BackFromPixelMini = drive.trajectorySequenceBuilder(startPose)
                .back(5)
                .build();

        // Set BackFromPixel to back backFromPixel
        TrajectorySequence BackFromPixelXS = drive.trajectorySequenceBuilder(startPose)
                .back(backFromPixelXS)
                .build();

        // Set CenterBoardTrajA to Center
        TrajectorySequence CenterBoardTrajA = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(-CenterOffsetDeg))
                .build();

        // Sat RightBoardTrajA to Right Side of Board
        TrajectorySequence RightBoardTrajA = drive.trajectorySequenceBuilder(startPose)
                .turn(Math.toRadians(rightOffsetDeg))
                .build();

        // Sat RightBoardTrajB to Right Side of Board
        TrajectorySequence RightBoardTrajB = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(rightOffset)
                .build();
        // Back 13
        TrajectorySequence Traj_X_BBS_FORWARDa = drive.trajectorySequenceBuilder(startPose)
                .back(X_BBS_FORWARDa)
                .build();
        // Strafe Left 15
        TrajectorySequence Traj_X_BBS_LEFTb = drive.trajectorySequenceBuilder(startPose)
                .strafeTo(new Vector2d(-5, 10))
                .build();

        // State initialization sequence is completed
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pos", rightHex.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        side = "Left";
        waitForStart();
        leftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightHex.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Squeeze Pixels
        gripServo1.setPosition(0);
        gripServo2.setPosition(1);

        // Update Telemetry
        telemetry.addData("Side", side);
        telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
        telemetry.addData("Pos", rightHex.getCurrentPosition());
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
            telemetry.addData("Pos", rightHex.getCurrentPosition());
            telemetry.update();
            // Place Pixel then turn back and go to board
            //strafe left
            drive.followTrajectorySequence(trajectory3b);
            sleep(700);

            //move arm down
            armServo.setPosition(0);
            sleep(700);
            drive.followTrajectorySequence(trajectory3bb);
            sleep(1100);

            //open purple side
            gripServo1.setPosition(1);
            sleep(700);

            //back up
            drive.followTrajectorySequence(BackFromPixelXS);
            sleep(700);

            //bring arm up
            armServo.setPosition(0.6);

            //turn towards board
            drive.followTrajectorySequence(turnAngleZ); // Turn Right
            drive.followTrajectorySequence(toBoardFromCenterTraj);
            drive.followTrajectorySequence(CenterBoardTrajA);
            drive.followTrajectorySequence(trajectory3b);
        } else {
            telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
            // Turn Right
            drive.followTrajectorySequence(turnAngleP);

            // Wait for Robot to settle
            sleep(1000);

            // If senses right, update telemetry and set side
            if (distanceFront.getDistance(DistanceUnit.INCH) < 10) {
                side = "Right";
                telemetry.addData("Side", side);
                telemetry.addData("Distance Front", distanceFront.getDistance(DistanceUnit.INCH));
                telemetry.addData("Pos", rightHex.getCurrentPosition());
                telemetry.update();

                drive.followTrajectorySequence(StrafeRightG);
                armServo.setPosition(0);
                sleep(700);
                drive.followTrajectorySequence(StrafeRightO);
                sleep(700);
                gripServo1.setPosition(1);
                sleep(500);
                drive.followTrajectorySequence(BackFromPixelMega);
                armServo.setPosition(0.6);
                drive.followTrajectorySequence(RightBoardTrajA);
                // Center on Right side of Board RightBoardTrajB
                drive.followTrajectorySequence(RightBoardTrajB);
            } else {

                // --- LEFT SIDE ---

                armServo.setPosition(0);
                drive.followTrajectorySequence(trajectory3c); // Fancy Moves
                // Drop Pixel
                gripServo1.setPosition(1);
                drive.followTrajectorySequence(BackFromPixelMini);
                drive.followTrajectorySequence(Traj_X_BBS_LEFTb);
                drive.followTrajectorySequence(trajectory3cb); // Turn
                armServo.setPosition(0.6);
            }
        }

        // Move toward the Board
        while(!toBoard && opModeIsActive()){

            // Set Power to Motors Based on Log Func
            double power = -0.75 * ((0.6 * Math.log10(distanceBack.getDistance(DistanceUnit.INCH) - DISTANCE_FROM_BACKBOARD)) + 0.2);

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

        // Stop Moving Base
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        leftHex.setPower(-1);
        rightHex.setPower(-1);
        sleep(1500);

        // move arm towards board
        while(!stopPower && opModeIsActive()){
            // Set Variables for easy access
            double power = pidController(position, leftHex.getCurrentPosition());
            double arm_position = leftHex.getCurrentPosition();

            // Check if within tolerable arm position
            if ((arm_position < (position + 30)) && (arm_position > (position - 30))) {
                // Break out of loop if tolerable position
                stopPower = true;
                break;
            }

            // Set power to hex motors
            leftHex.setPower(power);
            rightHex.setPower(power);
        }

        // Stop arm
        leftHex.setPower(0);
        rightHex.setPower(0);

        sleep(1000);

        // drop pixels
        gripServo2.setPosition(0);

        sleep(1000);

        // Move Arm back down
        while (opModeIsActive() && !stopPowerb) {
            telemetry.addData("Status", "Moving Arm Back Down...");
            telemetry.addData("Current Pos", leftHex.getCurrentPosition());
            telemetry.update();
            double power = pidController(positionb, leftHex.getCurrentPosition());
            if (((positionb + 30) > leftHex.getCurrentPosition()) && (leftHex.getCurrentPosition() > positionb - 30)) {
                stopPowerb = true;
            }
            leftHex.setPower(power);
            rightHex.setPower(power);
        }

        // Stop Arm Again
        leftHex.setPower(0);
        rightHex.setPower(0);

        // Put wrist down
        armServo.setPosition(0);
        drive.followTrajectorySequence(trajectory3bb);
    }
    public double pidController(double reference, double state) {
        double error = reference - state;
        integralSummation += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
