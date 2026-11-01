package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "rpmtsets", group = "Linear OpMode")
public class rpmtests extends LinearOpMode {
    private DcMotor motor = null;
    private double power = 0.5;


    @Override

    public void runOpMode(){
        motor = hardwareMap.get(DcMotor.class, "motor");


        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(power);
            if(gamepad1.a){
                power += 0.01;
                power = max(power, 1);
                telemetry.update();
                sleep(10);

            }
            if(gamepad1.b){
                power -= 0.01;
                power = min(power, 0);
                telemetry.update();
                sleep(10);
            }
        }
        motor.setPower(power);
        telemetry.addData("Motor Power: ","%4.2f",power);
        telemetry.update();
    }



}
