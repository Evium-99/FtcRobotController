package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Arm/Wrist Test")
public class armTest extends LinearOpMode {
    public static double armP = 0;
    public static double armI = 0.00004;
    public static double armD = 0.001;
    public double integralSummation = 0;
    public double lastError = 0;
    public static int position = -100;
    static int lastPosition = position;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        // Negative is Up
        // Positive is Down
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "wristMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor.setPositionPIDFCoefficients(0.5);
        waitForStart();
        while (opModeIsActive()) {
            if (lastPosition != position) {
                integralSummation = 0;
                timer.reset();
            }
            armMotor.setPower(pidController(position, armMotor.getCurrentPosition()));
            telemetry.addData("Position", armMotor.getCurrentPosition());
            telemetry.addData("Target", position);
            telemetry.update();
            lastPosition = position;
        }
    }
    public double pidController(double reference, double state) {
        double error = reference - state;
        integralSummation = error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        return (error * armP) + (derivative * armD) + (integralSummation * armI);
    }
}
