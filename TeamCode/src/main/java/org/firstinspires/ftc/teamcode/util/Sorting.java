package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;
import org.slf4j.helpers.NOPLogger;

@Config
public class Sorting implements  Updateable{
//---------MOTORS--------------------
    public Servo transfer_servo;
    public CRServo blade;
    public ColorSensor color_sensor_human;
    public ColorSensor color_sensor_intake;
    private Intake intake;
    public static PIDFCoefficients pidEliceCoefficients = new PIDFCoefficients(0.00022d,0.000003d,0.000002d,0);
    public static PIDFController pidElice = new PIDFController(pidEliceCoefficients);
    public DcMotorEx encoder_elice;
    private Telemetry telemetry;


//----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final int full_rotation = 8192;
    public static final int one_rotation = 1365;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // betwen [0, full_rotation] equal to position % full_rotation
    public int current_blade_index=0;
    public int last_blade_index=0;
    public double target;
    public double pidError;
    private ElapsedTime transferTimer;
    private boolean transfer_isUp = false;
    private boolean runPid= false;
    private int shooting_balls = 3;


//----------STATES-----------------
    public static COLORS[] magazine = new COLORS[7];// 2 - aruncare || 4 - intake  || 6 - rezerva
    public static int motif=6; // 6/4/2
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public Sorting(HardwareMap hwmap, Telemetry telemetry, Intake intake){
        blade = hwmap.get(CRServo.class, HardwareConfig.sorting);
        transfer_servo = hwmap.get(Servo.class,HardwareConfig.transfer);
        color_sensor_human = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_human);
        color_sensor_intake = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_intake);
        this.intake = intake;
        encoder_elice = hwmap.get(DcMotorEx.class, HardwareConfig.LB);
        this.telemetry = telemetry;




        transferTimer = new ElapsedTime();



        transfer_servo.setPosition(transfer_pos.val);
        encoder_elice.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder_elice.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder_elice.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
       // encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);

        clearMagazine();

    }

    public enum TRANSFER_POS{
        UP(0.8),DOWN(0);
        double val;
        TRANSFER_POS(double val) {
            this.val = val;
        }
    }
    public enum MOVING_STATES {
        WAITING_HUMAN_PLAYER, WAITING_INTAKE, NOTHING, MOVING , SHOOTING;
    }

    public static enum COLORS{
        GREEN("green"), PURPLE("parpal"), EMPTY("empty");
        public String val;
        COLORS(String val) {
            this.val = val;
        }
    }

    public void setNextState(MOVING_STATES state){
        next_moving_state = state;
        if(state==MOVING_STATES.WAITING_INTAKE){
            if (current_blade_index%2==1) {
                rotate_elice(0.5);
                return;
            }
        }
        else if (state==MOVING_STATES.WAITING_HUMAN_PLAYER) {
            if (current_blade_index%2==0) {
                if (magazine[6] == COLORS.EMPTY)
                    rotate_elice(0.5);
                else if (magazine[4] == COLORS.EMPTY)
                    rotate_elice(-0.5);
                else
                    rotate_elice(1.5);
                return;
            }
        }
        if(state==MOVING_STATES.SHOOTING)
            shooting_balls=3;

        pidElice.reset();
    }
    public void resetStates(){
        current_moving_state = MOVING_STATES.NOTHING;
        next_moving_state = MOVING_STATES.NOTHING;
        pidElice.reset();
    }
    public void resetEverything(){
        resetStates();
        last_moving_state = MOVING_STATES.NOTHING;
        last_blade_index = 0;
        current_blade_index = 0;
        runPid = false;
        transfer_ball(false);
        clearMagazine();
        pidError = 0;

        position = 0;
        blade_rotation = 0;
        shooting_balls = 3;
        target = 0;

        encoder_elice.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder_elice.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        encoder_elice.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);



    }
    public void transfer_ball(){ //toggle
        if(!transfer_isUp){
            transfer_servo.setPosition(TRANSFER_POS.UP.val);
            transferTimer.reset();
            transferTimer.startTime();
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
        }
        transfer_isUp = !transfer_isUp;

    }
    public void transfer_ball(boolean direction){ // true = up, false = down
        if(direction){
            transfer_servo.setPosition(TRANSFER_POS.UP.val);
            transferTimer.reset();
            transferTimer.startTime();
            transfer_isUp = true;
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
            transfer_isUp = false;
        }
    }

    public void rotate_elice(double turns){ // positive is right, negative is left
        last_moving_state = current_moving_state;
        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target = target + turns*one_rotation*2;
        pidElice.reset();
        pidElice.setTargetPosition(target);
        runPid = true;

    }

    @Override
    public void update(){
        position = -encoder_elice.getCurrentPosition();
        blade_rotation = (int)position % full_rotation;
        if(blade_rotation<0)blade_rotation=full_rotation+blade_rotation;
        // aici o sa punem functie citire pozitie

        if (runPid) { // abs(pos - target) > admissible error
            pidElice.updatePosition(position);
            double power = pidElice.run();
            pidError = pidElice.getError();
            blade.setPower(power);
        }
        else blade.setPower(0);

        /* if eroare > eps {
            runPid = true;
        }*/

//        current_blade_index = blade_rotation/one_rotation;

//        if (current_blade_index != last_blade_index){
//            if ( (position>0 && blade_rotation%one_rotation<one_rotation*0.79) || (position<0 && blade_rotation%one_rotation>one_rotation*0.79) ){
//                boolean direction_of_rotation;
//                if (current_blade_index==5&&last_blade_index==0)
//                    direction_of_rotation = false;
//                else if (current_blade_index==0&&last_blade_index==5)
//                    direction_of_rotation = true;
//                else
//                    direction_of_rotation =(current_blade_index-last_blade_index>0);
//
//                rotate_balls(direction_of_rotation);
//                last_blade_index = current_blade_index;
//
//            }
//        }


        if(transfer_isUp && transferTimer.milliseconds()>200){
            transfer_ball(false);
        }

        current_blade_index = pos_to_index((int)position);

        if (current_blade_index != last_blade_index){
                boolean direction_of_rotation;
                if (current_blade_index==5&&last_blade_index==0)
                    direction_of_rotation = false;
                else if (current_blade_index==0&&last_blade_index==5)
                    direction_of_rotation = true;
                else
                    direction_of_rotation =(current_blade_index-last_blade_index>0);

                rotate_balls(direction_of_rotation);
                last_blade_index = current_blade_index;


        }

        if ((current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE || current_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER)&& next_moving_state != MOVING_STATES.NOTHING){
            last_moving_state = current_moving_state;
            current_moving_state = next_moving_state;
            next_moving_state = MOVING_STATES.NOTHING;
            return;
        }
//MOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINGGGGGGGGGGGGGGGGGGGGGG
        if (current_moving_state == MOVING_STATES.MOVING){
//            if (next_moving_state != MOVING_STATES.NOTHING){
//                last_moving_state = current_moving_state;
//                current_moving_state = next_moving_state;
//                next_moving_state = MOVING_STATES.NOTHING;
//                return;
//            }

            if ((pidError>0?pidError:-pidError) < full_rotation/100){
            //if ((position>0?position:-position)>(target>0?target:-target)){
                pidElice.reset();
                runPid = false;
                //rotate_elice(0) posibil sa ajute la un moment dat;
                if (last_moving_state==MOVING_STATES.WAITING_INTAKE || last_moving_state==MOVING_STATES.WAITING_HUMAN_PLAYER || last_moving_state==MOVING_STATES.SHOOTING) {
                    current_moving_state = last_moving_state;
                    return;
                }
                current_moving_state = MOVING_STATES.NOTHING;

            }
        }
//IIIIIIIIIIIINNNNNNNNNNNNNNNNNNNNNNTTTTTTTTTTTTTTTTTTTAAAAAAAAAAAAAAAAAAKKKKKKKKKKKKKKKEEEEEEEEEEE
        if (current_moving_state == MOVING_STATES.WAITING_INTAKE){

            if (isFull()){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                return;
            }
            COLORS current_color = ColorFunctions.getColor(color_sensor_intake.red(),color_sensor_intake.green(),color_sensor_intake.blue());
            if (current_color != COLORS.EMPTY){
                magazine[4] = current_color;
                if (magazine[2] == COLORS.EMPTY)
                    rotate_elice(-1);
                else if (magazine[6] == COLORS.EMPTY)
                    rotate_elice(1);
            }
        }
//HUUUUUUUUUUUMMMMMMMMMMMMMAAAAAAAAAANNNNNNNNNNNNNNNNNNNNNNN
        if (current_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER){
            if (isFull()){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                if (current_blade_index%2==1)
                    rotate_elice(0.5);
                return;
            }

            COLORS current_color = ColorFunctions.getColor(color_sensor_human.red(),color_sensor_human.green(),color_sensor_human.blue());
            if (current_color != COLORS.EMPTY){
                magazine[5] = current_color;
                if (magazine[3] == COLORS.EMPTY)
                    rotate_elice(1);
                else if (magazine[1] == COLORS.EMPTY)
                    rotate_elice(-1);

            }
        }
        //SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTIIIIIIIIIIIIIIIIIIIIIINNNNNNNNNNNNNNNNNNNGGGGGGGGG
        if (current_moving_state == MOVING_STATES.SHOOTING){
            if(shooting_balls<=0){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                return;
            }
            shooting_balls--;
            transfer_ball(true);
            rotate_elice(1);

        }
    }
    public void sort_balls(){
        int current_green_pos = getGreen();
        int rotations = current_green_pos-motif;

        rotate_elice((double)rotations/2);
    }
    public void shoot(){
        if (motif != getGreen())
            sort_balls();
        setNextState(MOVING_STATES.SHOOTING);
    }

    public static int pos_to_index(int position){
        if (position<0) position = -position;
        position %= full_rotation;

        int index=0, min_dif=10000000, dif=0;
        for (int i=0;i<=6;i++){
            dif = position-full_rotation/6*i;
            if(dif<0)dif=-dif;

            if (dif<min_dif){
                min_dif = dif;
                index = i;
            }
        }
        if (index==6) index = 0;
        return index;
    }

    public static void rotate_balls(boolean right){
        COLORS aux;
        if (right){
            for(int i=6;i>=2;i--){
                aux = magazine[i];
                magazine[i] = magazine[i-1];
                magazine[i-1] = aux;
            }
        }
        else {
            for(int i=2;i<=6;i++){
                aux = magazine[i-1];
                magazine[i-1] = magazine[i];
                magazine[i] = aux;
            }

        }

    }
    public boolean isFull(){
        if((magazine[2] != COLORS.EMPTY && magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY) || (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)){
            intake.toggle(false);
            return true;
        } else {
            return false;
        }
    }
    public static int getGreen(){
        for(int i=1;i<=6;i++){
            if (magazine[i]==COLORS.GREEN)
                return i;
        }
        return 0;
    }
    public void clearMagazine(){
        magazine[1] = COLORS.EMPTY;
        magazine[2] = COLORS.EMPTY;
        magazine[3] = COLORS.EMPTY;
        magazine[4] = COLORS.EMPTY;
        magazine[5] = COLORS.EMPTY;
        magazine[6] = COLORS.EMPTY;
    }


    public  void telemetryData(){
        telemetry.addData("Absolut bladep pos: ",encoder_elice.getCurrentPosition());
        telemetry.addData("Moded blade pos: ",blade_rotation);

        telemetry.addData("Current_blade_index", current_blade_index);
        telemetry.addData("Pos_to_index",pos_to_index((int)position));
        telemetry.addData("blade_rotation % one_rotation",blade_rotation%one_rotation);
        telemetry.addData("Target: ",target);
        telemetry.addData("Pid error",pidError);
        telemetry.addData("Timer: ",transferTimer.milliseconds());

        telemetry.addData("1: ", magazine[1].val);
        telemetry.addData("2: ", magazine[2].val);
        telemetry.addData("3: ", magazine[3].val);
        telemetry.addData("4: ", magazine[4].val);
        telemetry.addData("5: ", magazine[5].val);
        telemetry.addData("6: ", magazine[6].val);



    }


}
