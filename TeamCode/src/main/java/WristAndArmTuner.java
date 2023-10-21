import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name="Wrist and Arm Tuner")
public class WristAndArmTuner extends LinearOpMode {
    public static double MIN = -1; // in
    public static double MID = 0; // in
    public static double MAX = 1; // in

    private Servo gripServo = null; // Grip Servo
    private Servo armServo = null; // Arm Servo

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Control Hub Servos
        gripServo = hardwareMap.get(Servo.class, "grip");
        armServo = hardwareMap.get(Servo.class, "arm");
        waitForStart();

        while (opModeIsActive()) {
            double t = getRuntime();
            while (getRuntime() - t < 2 && opModeIsActive()) {
                gripServo.setPosition(MIN);
                telemetry.addData("Value", MIN);
                telemetry.update();
            }
            t = getRuntime();
            while (getRuntime() - t < 2 && opModeIsActive()) {
                gripServo.setPosition(MID);
                telemetry.addData("Value", MID);
                telemetry.update();
            }
            t = getRuntime();
            while (getRuntime() - t < 2 && opModeIsActive()) {
                gripServo.setPosition(MAX);
                telemetry.addData("Value", MAX);
                telemetry.update();
            }
        }
    }
}
