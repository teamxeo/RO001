/** Team RO001 Xeo TELE-OP code 2017-2018 Relic Recovery*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@TeleOp(name = "Xeo17-18: Tele-OP", group = "Xeo17-18")
//@Disabled
public class XeoTeleOP extends OpMode {

    // declarare variabile
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorRidicare;
    DcMotor motorRelic;
    Servo   servoRidS_0;
    Servo   servoRidS_1;
    Servo   servoRidD_0;
    Servo   servoRidD_1;
    Servo   relicPrindere;
    Servo   relicRidicare;
    //IntegratingGyroscope gyro;
   // NavxMicroNavigationSensor navxMicro;
    Servo   servoBile;

    private double servoPoz = 0.68;
    private double servoPozS = 0.32;
    private double servoPozD = 0.62;
    private double motorPower = 1.0;
    private double relicMotorPower = 1;
    double timp;
    private int profil;
    private double relicPoz = 1;
    private double ridicarePoz = 1;
    private final int kMaxNumberOfMotors = 4;
    private final double maxOutput = 1.0;
    double alpha = 0, beta = 0;
    double xDir = 0, yDir = 0;
    boolean apasat = false;
    double pozDeschidere = 0.6792;
    double pozInchidere = 0.5926;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();


    /**
     * Constructor
     */
    public XeoTeleOP() {

    }


    @Override
    public void init() {        //functie init, se apeleaza la apasarea butonului init de pe driver station

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        motorRelic = hardwareMap.dcMotor.get("motor relic");
        //These work without reversing (Tetrix motors).
        //AndyMark motors may be opposite, in which case uncomment these lines:
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        servoRidS_0 = hardwareMap.servo.get("servo rid s 0");
        servoRidD_0 = hardwareMap.servo.get("servo rid d 0");
        servoRidS_1 = hardwareMap.servo.get("servo rid s 1");
        servoRidD_1 = hardwareMap.servo.get("servo rid d 1");
        motorRidicare = hardwareMap.dcMotor.get("motor rid");
        servoRidD_0.setPosition(servoPozD);
        servoRidS_0.setPosition(servoPozS);
        servoRidD_1.setPosition(servoPozD);
        servoRidS_1.setPosition(servoPozS);
       // relicPrindere=hardwareMap.servo.get("servo relic prindere");
       // relicRidicare=hardwareMap.servo.get("servo relic ridicare");
       // relicPrindere.setPosition(0);
       // relicRidicare.setPosition(0);
      //  navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
       // gyro = (IntegratingGyroscope)navxMicro;
        profil=1;
        servoBile = hardwareMap.servo.get("servo bile");
        servoBile.setPosition(1);

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //Wait until the gyro calibration is complete
      //  timer.reset();
      //  while (navxMicro.isCalibrating())  {
      //      telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
      //      telemetry.update();
     //   }
      //  telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

    }



    @Override
    public void loop() {        //functie loop, se repeta continuu, de la apasarea start pana la apasarea stop

        servoBile.setPosition(1);
        /** ----------------------------- <miscare> ----------------------------- */
        /* left stick controls direction
           right stick X controls rotation */

       // Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


         //   float unghi = angles.firstAngle;
          //  float var_1 = (float)Math.cos(unghi);
          //  float var_2 = (float)Math.sin(unghi);
            float gamepad1LeftY = -gamepad1.left_stick_y ;
            float gamepad1LeftX = -gamepad1.left_stick_x ;
            float gamepad1RightX = gamepad1.right_stick_x;

            /*--> if(gamepad1RightX < -0.1 || gamepad1RightX > 0.1){
                apasat = true;
            }
            if(gamepad1RightX >= -0.1 && gamepad1RightX <= 0.1 && apasat){
                apasat = false;
                alpha = unghi;
            }

            if(Math.abs(gamepad1LeftX) > 0.1 || Math.abs(gamepad1LeftY) > 0.1) {
                if (gamepad1LeftX != 0) {
                    beta = gamepad1LeftY / gamepad1LeftX;
                    xDir = (Math.cos(alpha) + Math.cos(beta)) / 2;
                    yDir = (Math.sin(alpha) + Math.sin(beta)) / 2;
                } else {
                    xDir = Math.cos(alpha);
                    yDir = Math.sin(alpha);
                }
            } else{
                xDir = 0;
                yDir = 0;
            } */

            /*double FrontLeft = -yDir - xDir - gamepad1RightX;
            double FrontRight = yDir - xDir - gamepad1RightX;
            double BackRight = yDir + xDir - gamepad1RightX;
            double BackLeft = -yDir + xDir - gamepad1RightX; */

            // holonomic formulas

            float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
            float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

            // clip the right/left values so that the values never exceed +/- 1
            FrontRight = Range.clip(FrontRight, -1, 1);
            FrontLeft = Range.clip(FrontLeft, -1, 1);
            BackLeft = Range.clip(BackLeft, -1, 1);
            BackRight = Range.clip(BackRight, -1, 1);

            // write the values to the motors
            motorFrontRight.setPower(FrontRight * motorPower);
            motorFrontLeft.setPower(FrontLeft * motorPower);
            motorBackLeft.setPower(BackLeft * motorPower);
            motorBackRight.setPower(BackRight * motorPower);



            //mecanumDrive_Cartesian(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,angles.firstAngle, motorPower);

            //cutie viteze miscare
            if (gamepad1.y) {
                motorPower = 0.5;
            }
            if (gamepad1.a) {
                motorPower = 0.15;
            }
            if (gamepad1.x) {
                motorPower = 0.3;
            }

        /* ----------------------------- </miscare> ----------------------------- */

            //telemetry.addLine();
            //telemetry.addData("Servo Bile:", servoBile);

        /** ---------------------------- <profiluri> ----------------------------  */
        if (gamepad2.y)
        {
            profil=1;
        }
      //  if (gamepad2.a)
       // {
      //      profil=2;
      //  }
        /* ---------------------------- </profiluri> ----------------------------  */
        /** ----------------------------- <prindere> ----------------------------- */

                servoRidD_0.setPosition(servoPozD);
                servoRidS_0.setPosition(servoPozS);
                servoRidD_1.setPosition(servoPozD);
                servoRidS_1.setPosition(servoPozS);


        if(gamepad2.a) { // inchidere
            servoPozS = 0.2988;
            servoPozD = 0.6411;
        }
        if(gamepad2.b) { // deschidere
            servoPozS = 0.4614;
            servoPozD = 0.4785;
        }
        if (profil==1)
        {
            if (gamepad2.left_trigger != 0 && servoPozD > 0.4 && servoPozS < 0.5) {
                servoPozS += gamepad2.left_trigger / 25;
                servoPozD -= gamepad2.left_trigger / 25;
            }
            if(gamepad2.right_trigger != 0  && servoPozS > 0.25 && servoPozD < 0.7 ){
                servoPozS -= gamepad2.right_trigger / 25;
                servoPozD += gamepad2.right_trigger / 25;
            }

        }


         /* ----------------------------- </prindere> ----------------------------- */

         /** ---------------------------- <ridicare> ----------------------------- */
        if (profil==1)
        {
            if (gamepad2.dpad_up)
            {
                motorRidicare.setPower(1);
            }
            if (gamepad2.dpad_down)
            {
                motorRidicare.setPower(-1);
            }
            if(!gamepad2.dpad_down && !gamepad2.dpad_up) {
                motorRidicare.setPower(0);
            }
        }

        telemetry.addData("Pozitie cod: ", servoPoz);
        telemetry.addData(" Dreapta: ",servoPozD);
        telemetry.addData(" Stanga: ", servoPozS);
         /* ----------------------------- </ridicare> ----------------------------- */

        /** ----------------------------- <relic> ----------------------------- */
        if (profil==2)
        {

            relicPrindere.setPosition(relicPoz);
            relicRidicare.setPosition(ridicarePoz);



            if(gamepad2.x)
                relicMotorPower = 0.5;
            if(gamepad2.b)
                relicMotorPower = 0.7;



            if(gamepad2.dpad_up)
                motorRelic.setPower(relicMotorPower);
              else
                if(gamepad2.dpad_down)
                    motorRelic.setPower(-relicMotorPower);
                       else
                    motorRelic.setPower(0);



            if (gamepad2.dpad_left  && relicPrindere.getPosition() < 1)
                relicPoz += 0.01;
            else if (gamepad2.dpad_right && relicPrindere.getPosition() > 0)
                relicPoz -= 0.01;

            if (gamepad2.left_bumper && relicRidicare.getPosition() < 1)
                ridicarePoz += 0.01;
            else if (gamepad2.right_bumper && relicRidicare.getPosition() > 0)
                ridicarePoz -= 0.01;
            telemetry.addData("gheara: ", relicPrindere.getPosition());
            telemetry.addData("relic: ", relicRidicare.getPosition());
        }

        telemetry.addData("Joystick X: ", gamepad1LeftX);
        telemetry.addData("Joystick Y: ", gamepad1LeftY);
        telemetry.addData("Alfa: ", alpha);
        telemetry.addData("Beta: ", beta);
       // telemetry.addData("Rotatie: ", angles.firstAngle);
        /* ---------------------------- </relic> ----------------------------- */
    }



    @Override
    public void stop() {        //functie stop, se apeleaza la apasarea butonului stop

        oprireServo();
        oprireMotoare();

    }


    public void oprireServo() {

        servoBile.close();
        servoRidD_0.close();
        servoRidS_0.close();
        servoRidD_1.close();
        servoRidS_1.close();
      //  relicPrindere.close();
      //  relicRidicare.close();

    }

    public void oprireMotoare() {

        motorBackRight.close();
        motorBackLeft.close();
        motorFrontRight.close();
        motorFrontLeft.close();
        motorRidicare.close();
       // motorRelic.close();

    }

    //joystick input scale
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
        double scaleInput (double dVal){
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle, double speed) {
        @SuppressWarnings("LocalVariableName")
        double xIn = x;
        @SuppressWarnings("LocalVariableName")
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compensate for gyro angle.
        double[] rotated = rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        double[] wheelSpeeds = new double[kMaxNumberOfMotors];
        wheelSpeeds[0] = -xIn - yIn - rotation;
        wheelSpeeds[1] = +xIn - yIn - rotation;
        wheelSpeeds[2] = xIn + yIn - rotation;
        wheelSpeeds[3] = -xIn + yIn - rotation;

        normalize(wheelSpeeds);
        motorFrontLeft.setPower(wheelSpeeds[0] * speed);
        motorFrontRight.setPower(wheelSpeeds[1] * speed);
        motorBackLeft.setPower(wheelSpeeds[2] * speed);
        motorBackRight.setPower(wheelSpeeds[3] * speed);
    }

        String formatRate ( float rate){
        return String.format("%.3f", rate);
    }

        String formatDegrees ( double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

        String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
     void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }


    }

