package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.concurrent.TimeUnit;

import static java.lang.System.arraycopy;
import static java.lang.System.currentTimeMillis;
import static java.lang.System.in;
@Autonomous (name = "Rotire Test", group =  "rotire test")
public class rotire_test extends LinearOpMode {
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

    double pozitieServoBileJos = 0.06; // de vazut
    double pozitieServoBileSus = 1;
    double vitezaIntoarcere = 0.07;
    double VitezaIntoarcereMare = 0.23;
    double bilaStanga = -10;
    double bilaDreapta = 10;
    double treshHold = 4;
    double vitezaMiscare = 0.1;
    int raft = 0;
    Servo servoBile;
    Servo servoBileSD;
    Servo CubSJ;
    Servo CubDJ;
    Servo CubSS;
    Servo CubDS;

    double rollInitial;
    double altInitial;
    double alpha;
    double beta = 0;
    double zeroRoll = 0;
    private boolean calibration_complete = false;
    double ServS = 0.26;
    double ServD = 0.67;
    double ServSLas = 0.4614;
    double ServDLas = 0.4785;
    boolean terminat;
    double distRaft1 = 47;
    double distRaft2 = 65;
    double distRaft3 = 83;
    double bilaAlbastra = 35000;
    double bilaRosie = 55000;
    static final int LED_CHANNEL = 5;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    //Servo servo = hardwareMap.servo.get("servo_jewel");
    AdafruitI2cColorSensor colorSensor;
    ColorSensor colorSensorPrioritar;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        terminat = false;
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
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        double position = 0;

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        colorSensor = (AdafruitI2cColorSensor) hardwareMap.get("color_sensor");
        colorSensorPrioritar = hardwareMap.colorSensor.get("color_sensor_2");

        servoBile = hardwareMap.servo.get("servo bile");
        servoBileSD = hardwareMap.servo.get("servo bile SD");

        CubSJ = hardwareMap.servo.get("servo rid s 0");
        CubDJ = hardwareMap.servo.get("servo rid d 0");
        CubSS = hardwareMap.servo.get("servo rid s 1");
        CubDS = hardwareMap.servo.get("servo rid d 1");
        mRid = hardwareMap.dcMotor.get("motor rid");
        rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.opticalDistanceSensor.get("sensor_distance");

        servoBileSD.setPosition(0.5);

        while (!calibration_complete && !Thread.currentThread().isInterrupted()) {
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
        }


        rollInitial = navx_device.getPitch();
        telemetry.addData("Roll: ", rollInitial);
        telemetry.addLine();
        altInitial = navx_device.getAltitude();

        navx_device.zeroYaw();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);


        while (!opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
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
                    raft = 3;
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

            telemetry.addData("poz servo", CubSJ.getPosition());
            telemetry.update();

        }


        while (opModeIsActive() && !terminat) {

            rotireTreptata(90,treshHold,vitezaIntoarcere);


            // TimeUnit.MILLISECONDS.sleep(2000);

            rotireTreptata(0,treshHold,vitezaIntoarcere);


            //rotire(90,1,0.04);
            terminat = true;
        }
        oprire();
        oprireTot();
    }


    public void oprire(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void rotireTreptata(double tinta, double treshHold, double viteza) throws InterruptedException {
        int incercari = 0;
        boolean suntPeTinta = false;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga;
            double mergiDreapta;

            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive()) {// cat timp sunt in dreapta tintei vreau sa merg in stanga
                viteza = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                mergiStanga = viteza;
                mergiDreapta = -viteza;
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
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)     // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= tinta - treshHold && opModeIsActive()) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta
                    viteza = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                    mergiStanga = viteza;
                    mergiDreapta = -viteza;
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
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;


            incercari++;
            /*if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }
            */

            if (suntPeTinta)
                oprire();

        }


    }


    public double actualizeazaViteza(double tinta, double unghiActual,int incercari){
        double diferenta = Math.abs(Math.abs(tinta) - Math.abs(unghiActual));
        double scadere = 0;
        if(incercari % 2 == 0) {
             scadere = incercari / 2 * 0.01;
        }
        else
            scadere = (incercari-1) / 2 * 0.01;

        if(diferenta <= 30){

            return vitezaIntoarcere - scadere;
        }
        else
            return VitezaIntoarcereMare;
    }

    public void oprireTot() {

        navx_device.close();
        rangeSensor.close();
        colorSensor.close();
        colorSensorPrioritar.close();
        hardwareMap.deviceInterfaceModule.get("dim").setDigitalChannelState(LED_CHANNEL, false);
        oprire();

    }

    public void rotireSpecialaTreptata(double tinta, double treshHold, double viteza) throws InterruptedException {

        int incercari = 0;
        boolean suntPeTinta = false;
        boolean amTrecut;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga;
            double mergiDreapta;
             mergiStanga = viteza;
             mergiDreapta = -viteza;
            amTrecut = false;
            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive() && !amTrecut) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                mergiStanga = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                mergiDreapta = -actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
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
                if (navx_device.getYaw() >= 0)
                    amTrecut = true;
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold) {    // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            }
            amTrecut = false;
            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= -tinta - treshHold && opModeIsActive() && !amTrecut) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta
                    mergiStanga = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                    mergiDreapta = -actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
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
                    if (navx_device.getYaw() >= -180 && navx_device.getYaw() <= -160)
                        amTrecut = true;
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;
            amTrecut = false;

            incercari++;
            /*if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }
            */

            if (suntPeTinta)
                oprire();

        }

    }
    public void rotire(double tinta, double treshHold, double viteza) throws InterruptedException {
        int incercari = 0;
        boolean suntPeTinta = false;
        while(!suntPeTinta && opModeIsActive()) {
            double mergiStanga = viteza;
            double mergiDreapta = -viteza;

            while(navx_device.getYaw() >= tinta + treshHold && opModeIsActive()){      // cat timp sunt in dreapta tintei vreau sa merg in stanga
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
            TimeUnit.MILLISECONDS.sleep(500);


            if( navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)     // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            if(!suntPeTinta){     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while(navx_device.getYaw() <= tinta - treshHold && opModeIsActive()) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

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
                TimeUnit.MILLISECONDS.sleep(500);
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
}

