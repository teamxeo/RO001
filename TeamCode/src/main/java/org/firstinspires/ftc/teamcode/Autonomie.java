package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
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

/**
 * Created by (for example John) on 2/14/2018.
 */

@Autonomous(name = "Autonomie", group = "Sensor")
public class Autonomie extends LinearOpMode {

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
    double stanga = -0.6411;
    double dreapta = 0.2988;
    double pozitieServoBileJos = 0.06; // de vazut
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
    Servo CubSJ;
    Servo CubDJ;
    Servo CubSS;
    Servo CubDS;
    double rollInitial;
    double altInitial;
    double vitezeHolonom[] = new double[4];
    double alpha;
    double beta = 0;
    double zeroRoll = 0;
    private boolean calibration_complete = false;
    double ServS = 0.29;
    double ServD = 0.64;
    double ServSLas = 0.47;
    double ServDLas = 0.14;
    boolean terminat ;
    double distRaft1 = 137;
    double distRaft2 = distRaft1 + 19;
    double distRaft3 = distRaft2 + 19;
    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    //Servo servo = hardwareMap.servo.get("servo_jewel");
    ColorSensor colorSensor;

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
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        servoBile = hardwareMap.servo.get("servo bile");
        CubSJ = hardwareMap.servo.get("servo rid s 0");
        CubDJ = hardwareMap.servo.get("servo rid d 0");
        CubSS = hardwareMap.servo.get("servo rid s 1");
        CubDS = hardwareMap.servo.get("servo rid d 1");
        mRid = hardwareMap.dcMotor.get("motor rid");
        rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.opticalDistanceSensor.get("sensor_distance");

        /*CubSJ.setPosition(ServS);
        CubDJ.setPosition(ServD);
        CubSS.setPosition(ServS);
        CubDS.setPosition(ServD);*/

        /*while (colorSensor.blue()==colorSensor.red() && position>0)
        {

            servoBile.setPosition(position);
            position += 0.01;

        }*/

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



        while (opModeIsActive() && ! terminat) {

            zeroRoll = navx_device.getPitch();
            prindeCub(ServS,ServD);
            servoBile.setPosition(pozitieServoBileJos);
            TimeUnit.MILLISECONDS.sleep(1000);
            telemetry.addData("Rotatie: ", navx_device.getYaw());
            telemetry.update();


            if (colorSensor.blue() < colorSensor.red()) {

                calibrareRampa(bilaDreapta ,  treshHold, vitezaIntoarcere);
                servoBile.setPosition(1);
                TimeUnit.MILLISECONDS.sleep(500);
                calibrareRampa(0 , treshHold , vitezaIntoarcere);

               // rotire(0 ,treshHold, vitezaIntoarcere);


            } else if (colorSensor.red() < colorSensor.blue()) {

                calibrareRampa(bilaStanga, treshHold, vitezaIntoarcere);      // am dat jos bila rosie
                puneServo(1);
                TimeUnit.MILLISECONDS.sleep(500);
                //rotire(0 ,treshHold, vitezaIntoarcere);       // ma pun pe 0 grade
                calibrareRampa(0,treshHold,vitezaIntoarcere);
            }
            else {
                puneServo(1);
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);
           // formuleHolonom(navx_device.getYaw(),beta,vitezeHolonom);

            da_teJosDePeRampa(vitezaMiscare , zeroRoll, altInitial);
            rotire(-90,treshHold,vitezaIntoarcere);
            du_teLaRaft2(raft,vitezaMiscare-0.01);
           // du_teLaRaft(raft,vitezaMiscare-0.01);
           // mergiFata(vitezaMiscare);
            TimeUnit.MILLISECONDS.sleep(300);
            //mergiDreapta(vitezaMiscare);
          //  TimeUnit.MILLISECONDS.sleep(500);
          //  mergiFata(-vitezaMiscare, 0);
         //   TimeUnit.MILLISECONDS.sleep(250);
          //  rotire(-92,treshHold,vitezaIntoarcere);
            oprire();

            mergiFata(vitezaMiscare, 0.03);
            TimeUnit.MILLISECONDS.sleep(2000);
            oprire();
            lasaCub(ServSLas,ServDLas);
            TimeUnit.MILLISECONDS.sleep(500);
            mergiFata(-vitezaMiscare, 0);
            TimeUnit.MILLISECONDS.sleep(1000);
            mRid.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(500);
            mRid.setPower(0);
            oprire();

            oprireTot();


            //telemetry.addData("Blue ", colorSensor.blue());
            //telemetry.addData("Red ", colorSensor.red());
            //telemetry.addLine();
            //telemetry.addLine(Double.toString(servo.getPosition()));
            terminat = true;
        }
        oprireTot();
    }

    public void calibrareRampa(double tinta , double treshHold ,double viteza) {


            double mergiStanga = viteza;
            double mergiDreapta = -viteza;

            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive()) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga



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


            while(navx_device.getYaw() <= tinta - treshHold && opModeIsActive()) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

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


    public void mergiFata(double viteza, double plus){
        motorFrontLeft.setPower(-viteza);
        motorFrontRight.setPower(viteza + plus);
        motorBackRight.setPower(viteza );
        motorBackLeft.setPower(-viteza - plus);
    }

    public void mergiDreapta(double viteza){
        motorFrontLeft.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackRight.setPower(-viteza);
        motorBackLeft.setPower(-viteza);
    }

    public void du_teLaRaft(int raft, double viteza) throws InterruptedException{

         double dist1 = rangeSensor.getDistance(DistanceUnit.CM);
         double dist2 = dist1;
         mergiFata(viteza, 0);
         do{

             telemetry.addLine();
             telemetry.addData("distanta initiala:", dist1);
             telemetry.addLine();
             telemetry.addData("distanta curenta:", dist2);
             telemetry.update();

             dist2 = rangeSensor.getDistance(DistanceUnit.CM);

             if(dist1 - dist2 > 5) {

                 raft--;
                 TimeUnit.MILLISECONDS.sleep(1000);
                 telemetry.addData("sunt la raftul:", raft);
                 telemetry.update();
             }

         }while(raft > 0 && opModeIsActive());

         oprire();


    }

    public void oprireTot() {

        navx_device.close();
        rangeSensor.close();
        colorSensor.close();
        oprire();

    }

    public void prindeCub(double stanga, double dreapta) throws InterruptedException {

        CubSJ.setPosition(stanga);
        CubDJ.setPosition(dreapta);
        TimeUnit.MILLISECONDS.sleep(500);
        mRid.setPower(1);
        TimeUnit.MILLISECONDS.sleep(500);
        mRid.setPower(0);

    }

    public void lasaCub(double stanga, double dreapta) throws InterruptedException {
        CubSJ.setPosition(stanga);
        CubDJ.setPosition(dreapta);
    }

    public void puneServo(double poz){
            servoBile.setPosition(poz);
    }



    public void da_teJosDePeRampa(double viteza, double rollInitial, double altInitial){

       mergiFata(viteza, 0);

        double treshHold = .2;
        while (Math.abs(navx_device.getPitch()) <= 10 && opModeIsActive()) {idle();}
        while(Math.abs(navx_device.getPitch()) >= 4 && opModeIsActive()) {
            telemetry.addData("Roll: ", Math.abs(navx_device.getPitch()));
            telemetry.update();
        }
        oprire();
    }

    public void puneServoTreptat(Servo servo , double tinta, double pozInitial){

        while(servo.getPosition() > tinta) {
            servo.setPosition(pozInitial);
            pozInitial -= 0.005;
        }

    }
    public void du_teLaRaft2(int raft, double viteza){
        double tinta;
        switch(raft) {

            case 1:
                tinta=distRaft1;
                break;
            case 2:
                tinta=distRaft2;
                break;
            case 3:
                tinta=distRaft3;
                break;
            default:
                tinta=distRaft1;

        }

        double distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
        while(distantaCurenta < tinta)
        {
            distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
            mergiDreapta(viteza);
            telemetry.addData("distanta Curenta :",distantaCurenta);
                    telemetry.update();
        }
        oprire();
    }

    public void oprire(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }



}