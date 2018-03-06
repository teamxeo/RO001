package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.concurrent.TimeUnit;

import static java.lang.Thread.sleep;

@Autonomous(name = "Autonomie Albastru", group = "Autonomous")

public abstract class AutonomieAlbastru extends OpMode {

    public ModernRoboticsI2cRangeSensor rangeSensor;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    DcMotor mRid;

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    double stanga = 0.1;
    double dreapta = -0.1;
    double pozitieServoBileJos = 0; // de vazut
    double pozitieServoBileSus = 1;
    int rotireDreapta = 1;
    int rotireStanga = -1;
    double vitezaIntoarcere = 0.07;
    double bilaStanga = - 10;
    double bilaDreapta = 10;
    double treshHold = 2;
    double vitezaMiscare = 0.1;
    int raft = 0;
    Servo servoBile;
    Servo CubS;
    Servo CubD;
    double rollInitial;
    double altInitial;
    double vitezeHolonom[] = new double[4];
    double alpha;
    double beta = 0;
    double zeroRoll = 0;
    private boolean calibration_complete = false;
    double ServS = 0.5;
    double ServD = 0.5;
    double ServSLas = 0.32;
    double ServDLas = 0.61;

    VuforiaTrackable relicTemplate;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    //Servo servo = hardwareMap.servo.get("servo_jewel");
    ColorSensor colorSensor;

    VuforiaLocalizer vuforia;

    @Override
    public void init() {

        servoBile = hardwareMap.servo.get("servo bile");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcXkeIj/////AAAAmRV1vqYUzkRcn7rBLlUty9FuNYEzPAn4gEn+mrBl7eKeI3qA1oEWAVrY8uLi+jewlHe7i56Zoza2WB+sEq6MXpjOjQm4MCVCe3CmeVuCQEyVDU9QVGqoO1swzzY6x2k9yRoQgmuIW1BxEwxj4mNeFwPGZEsWJlofpFHnxuKNdO2DzQ0SeNAl+iksEodJBR1NKv22ORk0snNYX1u1b9dzsqZeq/ONXHhwm9KXDqCjDhwCndzm2oMHu7tASBsRTDjVJXnWTqVFsi9QQQqq2IX4Kxybhu/vck51l/f9gPItPE/YoVIL3UqyvKz5YDHpZbqrOYrpFLxtf8NFgNI/Aq5pNwcupiqayd5FM0hQzsWC2IHu";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        double position = 0;

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        CubS = hardwareMap.servo.get("servo rid s");
        CubD = hardwareMap.servo.get("servo rid d");
        mRid = hardwareMap.dcMotor.get("motor rid");
        rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.opticalDistanceSensor.get("sensor_distance");

        //servo.setPosition(position);

        /*while (colorSensor.blue()==colorSensor.red() && position>0)
        {

            servoBile.setPosition(position);
            position += 0.01;

        }*/




        rollInitial = navx_device.getPitch();
        telemetry.addData("Roll: ", rollInitial);
        telemetry.addLine();
        altInitial = navx_device.getAltitude();

        navx_device.zeroYaw();

        relicTrackables.activate();

    }

    @Override
    public void init_loop() {

        /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
        calibration_complete = !navx_device.isCalibrating();
        if (!calibration_complete) {
            telemetry.addData("navX-Micro", "Startup Calibration in Progress");
        }
        if (calibration_complete)
            telemetry.addData("navX-Micro", "M-am calibrat");
        telemetry.update();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        switch (vuMark) {

            case LEFT:
                raft = 1;
                break;
            case CENTER:
                raft = 2;
                break;
            case RIGHT:
                raft = 3;
                break;
            default:
                raft = 1;
                break;


        }

        telemetry.addData("Rotatie: ", navx_device.getYaw());
        telemetry.addLine();
        telemetry.addData("Albastru: ", colorSensor.blue());
        telemetry.addLine();
        telemetry.addData("Rosu: ", colorSensor.red());
        telemetry.addLine();
        telemetry.addData("distanta initiala:", rangeSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Altitudine: ", navx_device.getAltitude());
        telemetry.addData("Roll: ", navx_device.getPitch());
        telemetry.addLine();
        telemetry.addData("VuMark:", vuMark);

        telemetry.addData("poz servo", servoBile.getPosition());
        telemetry.update();

    }

    @Override
    public void start() {

        zeroRoll = navx_device.getPitch();

        telemetry.addData("Rotatie: ", navx_device.getYaw());
        telemetry.update();
        puneServo(pozitieServoBileJos);
        prindeCub(ServS, ServD);
        asteapta(1);
        if (colorSensor.blue() < colorSensor.red()) {

            calibrareRampa(bilaDreapta ,  treshHold, vitezaIntoarcere);
            servoBile.setPosition(1);
            calibrareRampa(0 , treshHold , vitezaIntoarcere);

            // rotire(0 ,treshHold, vitezaIntoarcere);


        } else if (colorSensor.red() < colorSensor.blue()) {

            calibrareRampa(bilaStanga, treshHold, vitezaIntoarcere);      // am dat jos bila rosie
            puneServo(1);
            //rotire(0 ,treshHold, vitezaIntoarcere);       // ma pun pe 0 grade
            calibrareRampa(0,treshHold,vitezaIntoarcere);
        }
        oprire();

        // formuleHolonom(navx_device.getYaw(),beta,vitezeHolonom);
        da_teJosDePeRampa(vitezaMiscare , zeroRoll, altInitial);

        rotire(0,treshHold,vitezaIntoarcere);
        du_teLaRaft(raft,vitezaMiscare);
        mergiFata(vitezaMiscare);
        asteapta(0.3);
        mergiDreapta(vitezaMiscare);
        asteapta(0.5);
        rotire(-85,treshHold,vitezaIntoarcere);
        oprire();
        mergiFata(vitezaMiscare);
        asteapta(1.0);
        oprire();
        lasaCub(ServSLas,ServDLas);
        mergiFata(-vitezaMiscare);
        oprire();


        //telemetry.addData("Blue ", colorSensor.blue());
        //telemetry.addData("Red ", colorSensor.red());
        //telemetry.addLine();
        //telemetry.addLine(Double.toString(servo.getPosition()));
        telemetry.update();

    }


    @Override
    public void stop() {

        oprire();
        navx_device.close();
        rangeSensor.close();
        colorSensor.close();


    }

    public void calibrareRampa(double tinta , double treshHold ,double viteza) {


        double mergiStanga = viteza;
        double mergiDreapta = -viteza;

        while (navx_device.getYaw() >= tinta + treshHold ) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga



            telemetry.addData("Rotatie: ", navx_device.getYaw());
            telemetry.addLine();
            telemetry.addData("treshhold:", treshHold);
            telemetry.addLine();
            telemetry.addData("viteza:", viteza);
            telemetry.update();



            motorFrontLeft.setPower(mergiStanga);
            motorFrontRight.setPower(mergiStanga);
            motorBackRight.setPower(mergiStanga);
            motorBackLeft.setPower(mergiStanga);
        }
        oprire();


        while(navx_device.getYaw() <= tinta - treshHold ) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

            telemetry.addData("Rotatie: ", navx_device.getYaw());
            telemetry.addLine();
            telemetry.addData("treshhold:", treshHold);
            telemetry.addLine();
            telemetry.addData("viteza:", viteza);
            telemetry.update();

            motorFrontLeft.setPower(mergiDreapta);
            motorFrontRight.setPower(mergiDreapta);
            motorBackRight.setPower(mergiDreapta);
            motorBackLeft.setPower(mergiDreapta);
        }
        oprire();

    }

    public void oprire(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void formuleHolonom(double alpha, double beta, double[] vitezeHolonom) {

        double xM, yM, scale=0.1;
        xM = ( Math.cos(alpha) + 1 )/2;
        yM = Math.sin(alpha)/2;
        vitezeHolonom[0] = scale*(-xM-yM);
        vitezeHolonom[1] = scale*(-xM+yM);
        vitezeHolonom[2] = scale*(xM+yM);
        vitezeHolonom[3] = scale*(+xM-yM);

    }

    public void urcaServo(Servo servo) {

        servo.setPosition(1);

    }


    public void rotire(double tinta, double treshHold, double viteza) {
        int incercari = 0;
        boolean suntPeTinta = false;

        while(!suntPeTinta ) {
            double mergiStanga = viteza;
            double mergiDreapta = -viteza;

            while(navx_device.getYaw() >= tinta + treshHold ){      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
            }
            oprire();
            asteapta(0.5);


            if( navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)     // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            if(!suntPeTinta){     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while(navx_device.getYaw() <= tinta - treshHold ) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                }
                oprire();
                asteapta(0.5);
            }

            if( navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;


            incercari++;
            if(incercari % 2 == 0 && incercari != 0){   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }

            if(suntPeTinta)
                oprire();

        }



    }


    public void mergiFata(double viteza){
        motorFrontLeft.setPower(-viteza);
        motorFrontRight.setPower(viteza);
        motorBackRight.setPower(viteza);
        motorBackLeft.setPower(-viteza);
    }

    public void mergiDreapta(double viteza){
        motorFrontLeft.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackRight.setPower(-viteza);
        motorBackLeft.setPower(-viteza);
    }

    public void du_teLaRaft(int raft, double viteza) {

        double dist1 = rangeSensor.getDistance(DistanceUnit.CM);
        double dist2 = dist1;
        mergiFata(viteza);
        do{

            telemetry.addLine();
            telemetry.addData("distanta initiala:", dist1);
            telemetry.addLine();
            telemetry.addData("distanta curenta:", dist2);
            telemetry.update();

            dist2 = rangeSensor.getDistance(DistanceUnit.CM);

            if(dist1 - dist2 > 5) {

                raft--;
                asteapta(1.0);
                telemetry.addData("sunt la raftul:", raft);
                telemetry.update();
            }

        }while(raft > 0 );

        oprire();


    }

    public void prindeCub(double stanga, double dreapta) {

        CubS.setPosition(stanga);
        CubD.setPosition(dreapta);
        asteapta(0.5);
        mRid.setPower(1);
        asteapta(0.5);
        mRid.setPower(0);

    }

    public void lasaCub(double stanga, double dreapta) {

        CubS.setPosition(stanga);
        CubD.setPosition(dreapta);
        mRid.setPower(-1);
        asteapta(0.5);
        mRid.setPower(0);

    }

    public void puneServo(double poz){
        servoBile.setPosition(poz);
    }
    public void da_teJosDePeRampa(double viteza, double rollInitial, double altInitial){

        mergiFata(viteza);

        double treshHold = .2;
        while (Math.abs(navx_device.getPitch()) <= 10) {}
        while(Math.abs(navx_device.getPitch()) >= 4) {
            telemetry.addData("Roll: ", Math.abs(navx_device.getPitch()));
            telemetry.update();
        }
        oprire();
    }

    public void asteapta(double target) {

        resetStartTime();
        while (getRuntime() <  target) {}

    }





}
