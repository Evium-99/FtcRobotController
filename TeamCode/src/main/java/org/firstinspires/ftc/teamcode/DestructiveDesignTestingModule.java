package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
public class DestructiveDesignTestingModule extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motor1 = null; // Front Right
    private DcMotor motor2 = null; // Front Left
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right

    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        motor2  = hardwareMap.get(DcMotor.class, "motor2");
        motor3  = hardwareMap.get(DcMotor.class, "motor3");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 10 seconds
        motor1.setPower(FORWARD_SPEED);
        motor2.setPower(FORWARD_SPEED);
        motor3.setPower(FORWARD_SPEED);
        motor4.setPower(FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < 10)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("Power", "Power: %4.1f", FORWARD_SPEED);
            telemetry.update();
        }

        // Step 2:  Stop
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
