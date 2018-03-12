package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by (for example John) on 1/11/2018.
 */
@Disabled
@TeleOp(name = "Teste in plm", group = "Concept")
public class teste_in_plm extends OpMode {


    DcMotor mot1;
    @Override
    public void init() {
        mot1 = hardwareMap.dcMotor.get("test");
    }

    @Override
    public void loop() {
        mot1.setPower(0.5);
    }

    @Override
    public void stop() {

    }


}
