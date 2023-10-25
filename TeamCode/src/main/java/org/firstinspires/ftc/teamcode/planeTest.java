package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name = "PlaneTesting")
public class planeTest extends LinearOpMode {
    private DcMotor MotorFL;
    @Override
    public void runOpMode() {
        MotorFL = hardwareMap.get(DcMotor.class, "MotorFL");
        waitForStart();
        while(opModeIsActive()) {
            MotorFL.setPower(1);
        }
    }
}
