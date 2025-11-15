package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;

public class Sorting implements  Updateable{
//---------MOTORS--------------------
    public Servo transfer_servo;
    public CRServo blade;
    public ColorSensor color_sensor_human;
    public ColorSensor color_sensor_intake;
    public PIDFCoefficients pidEliceCoefficients = new PIDFCoefficients(15,1,0.1,0);
    public PIDFController pidElice = new PIDFController(pidEliceCoefficients);


//----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final int full_rotation = 600;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // betwen [0, full_rotation] equal to position % full_rotation
    public double blade_index=0;

//----------STATES-----------------
    public double half_rotation=0;
    public static COLORS[] magazine = new COLORS[7]; // 2 - aruncare || 4 - intake  || 6 - rezerva
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public Sorting(HardwareMap hwmap, Telemetry telemetry){
        blade = hwmap.get(CRServo.class, HardwareConfig.sorting);
        transfer_servo = hwmap.get(Servo.class, HardwareConfig.transfer);
        color_sensor_human = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_human);

        magazine[0] = COLORS.EMPTY;
        magazine[1] = COLORS.EMPTY;
        magazine[2] = COLORS.EMPTY;


        transfer_servo.setPosition(transfer_pos.val);

    }

    public enum TRANSFER_POS{
        UP(1),DOWN(0);
        int val;
        TRANSFER_POS(int val) {
            this.val = val;
        }
    }
    public enum MOVING_STATES {
        WAITING_HUMAN_PLAYER, WAITING_INTAKE, NOTHING, MOVING;
    }

    public enum COLORS{
        GREEN, PURPLE, EMPTY;
    }

    public void setNextState(MOVING_STATES state){
        next_moving_state = state;
    }

    public void rotate_elice(double turns){ // positive is right, negative is left
        last_moving_state = current_moving_state;
        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        if (turns != (int)turns)
            if (turns>0)
                half_rotation +=0.5;
            else
                half_rotation-=0.5;
        rotate_balls(turns);
        if ((int)half_rotation!=0){
            half_rotation = 0;
        }
        double target = position / full_rotation * full_rotation + full_rotation/6 * pos_to_index((int)position) + full_rotation/6*turns*2;

        pidElice.reset();
        pidElice.setTargetPosition(target);

    }

    @Override
    public void update(){
        // aici o sa punem functie citire pozitie
        pidElice.updatePosition(position);
        double power = pidElice.run();
        if ((current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE || current_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER)&& next_moving_state != MOVING_STATES.NOTHING){
            last_moving_state = current_moving_state;
            current_moving_state = next_moving_state;
            next_moving_state = MOVING_STATES.NOTHING;
            return;
        }

        if (current_moving_state == MOVING_STATES.MOVING){
            if (next_moving_state != MOVING_STATES.NOTHING){
                last_moving_state = current_moving_state;
                current_moving_state = next_moving_state;
                next_moving_state = MOVING_STATES.NOTHING;
                return;
            }

            if (pidElice.getError() < full_rotation/100){
                if (last_moving_state==MOVING_STATES.WAITING_INTAKE) {
                    current_moving_state = last_moving_state;
                    return;
                }
                else if (last_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER){
                    current_moving_state = last_moving_state;
                    return;
                }
                current_moving_state = MOVING_STATES.NOTHING;
                pidElice.reset();
                //rotate_elice(0) posibil sa ajute la un moment dat;
            }
        }

        if (current_moving_state == MOVING_STATES.WAITING_INTAKE){
            if ((int)half_rotation !=half_rotation) {
                rotate_elice(half_rotation);
                return;
            }
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

        if (current_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER){
            if (isFull()){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                if ((int)half_rotation !=half_rotation)
                    rotate_elice(half_rotation);
                return;
            }
            if (half_rotation==0) {
                if (magazine[6] == COLORS.EMPTY)
                    rotate_elice(0.5);
                else if (magazine[4] == COLORS.EMPTY)
                    rotate_elice(-0.5);
                else
                    rotate_elice(1.5);
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
    }

    public static int pos_to_index(int position){
        if (position<0) position = -position;
        if (position >600) position %= full_rotation;

        int index=0, min_dif=10000, dif=0;
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

    public static void rotate_balls(double direction){
        COLORS aux;
        direction %= 3;
        while (direction>0.5 || direction < -0.5) {
            if (direction < 0) {
                aux = magazine[6];
                magazine[6] = magazine[4];
                magazine[4] = magazine[2];
                magazine[2] = aux;

                aux = magazine[5];
                magazine[5] = magazine[3];
                magazine[3] = magazine[1];
                magazine[1] = aux;
                direction++;
            } else {
                aux = magazine[2];
                magazine[2] = magazine[4];
                magazine[4] = magazine[6];
                magazine[6] = aux;

                aux = magazine[1];
                magazine[1] = magazine[3];
                magazine[3] = magazine[5];
                magazine[5] = aux;
                direction--;
            }
        }
        if (direction>0){
            aux = magazine[1];
            for(int i=1;i<6;i++)
                magazine[i] = magazine[i+1];
            magazine[6] = aux;
        }
        else if (direction <0) {
            aux = magazine[6];
            for (int i=6;i>1;i--)
                magazine[i] = magazine[i-1];
            magazine[1] = magazine[6];
        }

    }
    public static boolean isFull(){
        return (magazine[2] != COLORS.EMPTY && magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY) || (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY);
    }


}
