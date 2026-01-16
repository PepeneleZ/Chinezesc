package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.MOTIF;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.MOVING_STATES;

import org.firstinspires.ftc.teamcode.util.StorageSlot;



public class Storage implements Updateable {
    public StorageSlot[] storageSlots = new StorageSlot[4];
    public MOTIF motif = MOTIF.PPG;
    public MOVING_STATES state = MOVING_STATES.NOTHING;
    public Intake intake;
    private ElapsedTime transferTimer;
    private Telemetry telemetry;
    private int[] shooting_order = new int[4];
    public int shooting_index = 1;
    public int shift=0;

    public Storage(HardwareMap hardwareMap, Telemetry telemetry, Intake intake){
        for (int i=1;i<=3;i++){
            storageSlots[i] = new StorageSlot(hardwareMap,i);
        }
        this.intake = intake;
        this.telemetry = telemetry;
    }

    public void setState(MOVING_STATES state) {
        this.state = state;
        for(StorageSlot slot : storageSlots){
            if (state == MOVING_STATES.WAITING_INTAKE){
                slot.toggle_sensor(true);
            }
            else if (state == MOVING_STATES.NOTHING) {
                slot.toggle_sensor(false);
            }
            else if (state == MOVING_STATES.SHOOTING) {
                slot.toggle_sensor(false);
                shooting_index = 1;
                setShooting_order();
            }

        }
    }

    @Override
    public void update() {
        if (state == MOVING_STATES.WAITING_INTAKE){
            if (isFull()) {
                setState(MOVING_STATES.NOTHING);
                intake.toggle(Constants_Enums.INTAKE_STATES.STOPPED);
            }
        }
        else if (state==MOVING_STATES.SHOOTING){
            if (shooting_index==4 ) {
                setState(MOVING_STATES.NOTHING);
                return;
            }
            shooting_index = (shooting_order[shooting_index]==0?shooting_index+1:shooting_index);

            if (transferTimer.seconds()>0.2 || shooting_index==1){
                if (!storageSlots[shooting_index].isUP)
                    transfer_ball(shooting_order[shooting_index],true);
                else{
                    transfer_ball(shooting_order[shooting_index++],false);
                }

            }
        }

        for(StorageSlot slot : storageSlots){
            slot.update();
        }
    }

    public boolean isFull(){
        boolean isFull=true;
        for(StorageSlot slot : storageSlots){
            if (slot.color == COLORS.EMPTY) {
                isFull = false;
                break;
            }
        }
        return isFull;
    }

    public void transfer_ball(int index){ //toggle
        if(!storageSlots[index].isUP){
            storageSlots[index].toggle_transfer(true);

            transferTimer.reset();
            transferTimer.startTime();

            if(Turret.turret_launcher_state != Constants_Enums.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                storageSlots[index].color = COLORS.EMPTY;
            }
        }
        else{
            storageSlots[index].toggle_transfer(false);
            transferTimer.reset();
            transferTimer.startTime();

        }

    }
    public void transfer_ball(int index, boolean toggle){ //toggle
        if(toggle){
            storageSlots[index].toggle_transfer(true);

            transferTimer.reset();
            transferTimer.startTime();

            if(Turret.turret_launcher_state != Constants_Enums.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                storageSlots[index].color = COLORS.EMPTY;
            }
        }
        else{
            storageSlots[index].toggle_transfer(false);
            transferTimer.reset();
            transferTimer.startTime();

        }

    }
    public void setShooting_order(){
        int green_pos=0, current_motif = motif.val, cont_purple = 0, current_shift=shift%4; // i made a new variable (current_shift) in order to not affect the shift variable of the class
        int[] purple = new int[2];


        for(int i=1;i<=3;i++){
            shooting_order[i] = 0;
            if (storageSlots[i].color == COLORS.GREEN) {
                green_pos = i;
            }
            else if (storageSlots[i].color == COLORS.PURPLE)
                purple[cont_purple++] = i;
        }
        current_motif += shift;
        if (current_motif>3) {
            current_motif %= 4;
            current_motif++;
        }
        if (green_pos!=0) shooting_order[current_motif] = green_pos;
        cont_purple = 0;
        for(int i=1;i<=3;i++){
            if (i != current_motif ){
                shooting_order[i] = purple[cont_purple++];
            }
        }
    }

    private void telemetryData(){
        telemetry.addData("Shooting index: ",shooting_index);
        telemetry.addData("Moving state: ",state.val);
        for(int i=1;i<=3;i++){
            telemetry.addData("COLOR: -> Storage "+i+": ",storageSlots[i].color.val);
        }
        //I made two for loops so it would look better
        for(int i=1;i<=3;i++){
            telemetry.addData("Shooting order "+i+": ",shooting_order[i]);
        }
        telemetry.addLine("-----------------------------------------------------"); // separate the mechanisms to make the text easier to read
    }





}