package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.MultiplePIDF;
import org.firstinspires.ftc.teamcode.lib.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.Constants.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants.INTAKE_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.MOVING_STATES;
import org.firstinspires.ftc.teamcode.util.Constants.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants.MOTIF;



//     (. Y .)
//       ).(
//      \ Y /
//       | |


@Config
public class Sorting implements  Updateable{
    //---------MOTORS--------------------
    public Servo transfer_servo1, transfer_servo2;
    public DcMotorEx sorting_encoder;
    public CRServo blade1, blade2;
    public ColorSensor color_sensor_intake;
    private final Intake intake;
    private final VoltageSensor voltageSensor;
    //public static PIDF pidElice = new PIDF(0.000163d,0.0000062d,0.00000091d);
    public static MultiplePIDF pidElice = new MultiplePIDF(0.0001d,0.000002d,0.000000d,0);

    private final Telemetry telemetry;


    //----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final double full_rotation = 8192;
    public static final double one_rotation = full_rotation/6.0;
    private static final double durationOfOneRotation = 0.5;
    private double slightlyMovingDuration=0;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // between [0, full_rotation] equal to position % full_rotation
    public int current_blade_index=0;
    public int last_blade_index=0;
    public int shift=0;
    public int shooting_index=1;
    public double target;
    public double pidError;
    private final ElapsedTime transferTimer, collectTimer, slightlyMovingTimer,safe_timer,moving_timer;
    private boolean transfer_isUp = false;
    private int shooting_balls = 0;
    private static final double admissible_error=100;//140
    private boolean runPid=false;
    public double manualDeviation = 0;
    private double deltaManualDeviation = 0;

    private double startOfManualDeviation = 0;
    public boolean respectMotif = true;
    private boolean movedball = false;
    public boolean shotBall = false;
    public static double kF_fornrofballs=1e-10;
    public double collectTime = 0;
    private double rotations_for_telemetry;
    private double niggacuplus=0;
    private double niggacuminus=0;

    //momentul de frecare e momentu care se opune momentului motorului (care misca elicea)
    //
    //----------STATES-----------------
    public static COLORS[] magazine = new COLORS[7];// 2 - intake || 4 - aruncare  || 6 - rezerva   1 - intrare human
    public static COLORS[] shooting_order = new COLORS[4];
    public double[] shooting_order_powers = new double[5];
    public static MOTIF motif = MOTIF.PPG;
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public Sorting(HardwareMap hwmap, Telemetry telemetry, Intake intake, VoltageSensor voltageSensor){
        blade1 = hwmap.get(CRServo.class, HardwareConfig.sorting1);
        blade2 = hwmap.get(CRServo.class, HardwareConfig.sorting2);
        sorting_encoder = hwmap.get(DcMotorEx.class, HardwareConfig.right_lifter);

        transfer_servo1 = hwmap.get(Servo.class, HardwareConfig.transfer_servo1);
        transfer_servo2 = hwmap.get(Servo.class, HardwareConfig.transfer_servo2);
        color_sensor_intake = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_intake);
        this.voltageSensor = voltageSensor;
        this.intake = intake;
        this.telemetry = telemetry;


        pidElice.addPidCoefficients(0.0002,0.000002d,0.000000d);// pentru o rotatie jumate (3/6
        pidElice.addPidCoefficients(0.000163d,0.0000062d,0.00000091d);// pentru jumate de rotatie (1/6)



        transferTimer = new ElapsedTime();
        collectTimer = new ElapsedTime();
        slightlyMovingTimer = new ElapsedTime();
        safe_timer = new ElapsedTime();
        moving_timer = new ElapsedTime();

        //transfer_servo1.setDirection(Servo.Direction.FORWARD);
        transfer_servo1.setDirection(Servo.Direction.REVERSE);

        transfer_servo1.setPosition(0);
//        transfer_servo2.setPosition(0);


        //transfer_servo1.setPosition(transfer_pos.val);
        //transfer_servo2.setPosition(transfer_pos.val);

        blade1.setDirection(DcMotorSimple.Direction.FORWARD);
        blade2.setDirection(DcMotorSimple.Direction.FORWARD);

        sorting_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorting_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        blade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        blade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blade.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
         */

        // encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);

        for (int i=0;i<=3;i++)
            shooting_order[i] = COLORS.EMPTY;

        pidElice.resetPid();
        clearMagazine();

    }



    public void setNextState(MOVING_STATES state) {
        //        if(state == MOVING_STATES.SHOOTING && Turret.turret_launcher_state == Turret.TURRET_LAUNCH_SPEEDS.STOPPED)
        //            return;
        if (current_moving_state == MOVING_STATES.MOVING) {
            next_moving_state = state;
            return;
        }
        else if (state == MOVING_STATES.NOTHING){
            current_moving_state = state;
            last_moving_state = MOVING_STATES.NOTHING;
            return;
        }
        else
            current_moving_state = state;


        if (state == MOVING_STATES.WAITING_INTAKE) {
            collectTimer.reset();
            collectTimer.startTime();
            collectTime = 0;

//              if (current_blade_index % 2 == 1)
//                  rotate_elice(0.5);
        }
        else if (state == MOVING_STATES.SHOOTING){
//            if(Math.abs(pidError)<admissible_error*1.1){ // se poate schimba valorea
//                current_moving_state = MOVING_STATES.NOTHING;
//                last_moving_state = MOVING_STATES.NOTHING;
//            }
            shooting_index = 1;
            //shooting_balls = getNumberOfBalls();
            if (shooting_balls!=3)
                shooting_order = getShooting_order();
            if (current_blade_index % 2 == 0) {
                if (magazine[6] == shooting_order[1])
                    rotate_elice(-0.5);
                else
                    rotate_elice(0.5);
            }
            //makeShootingOrder();
            if (getNumberOfBalls() == 0 && shooting_balls !=3)
                current_moving_state = MOVING_STATES.NOTHING;

        }
    }


    public void manualBlade(){
        if (startOfManualDeviation==0) startOfManualDeviation = position;
        else deltaManualDeviation = position-startOfManualDeviation;
        if (Math.abs(deltaManualDeviation)>=one_rotation){
            target += one_rotation * ((deltaManualDeviation>0)?-1:1);
            deltaManualDeviation-=one_rotation;
        }
        pidElice.setTargetPosition(position);
        runPid = false;
        blade1.setPower(manualDeviation);
        blade2.setPower(manualDeviation);
    }
    public void resetStates(){
        current_moving_state = MOVING_STATES.NOTHING;
        next_moving_state = MOVING_STATES.NOTHING;
        pidElice.resetPid();
    }
    public void resetManual(){
        manualDeviation = 0;
        deltaManualDeviation = 0;
        startOfManualDeviation = 0;
    }
    public void resetEverything(){
        resetStates();
        pidElice.setTargetPosition(0);
        pidElice.resetPid();
        last_moving_state = MOVING_STATES.NOTHING;
        last_blade_index = 0;
        current_blade_index = 0;
        //isManually = false;
        resetManual();
        runPid = false;
        transfer_ball(false);
        transfer_isUp = false;
        clearMagazine();
        pidError = 0;

        position = 0;
        blade_rotation = 0;
        target = 0;
        shooting_balls = 0;
        target = 0;
        pidError = 0;
        intake.toggle(INTAKE_STATES.STOPPED);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);

        sorting_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);



    }
    public void transfer_ball(){ //toggle
        if (current_blade_index % 2 == 0) {
            return;
        }
        if(!transfer_isUp){
            transfer_servo1.setPosition(0.4);
//            transfer_servo2.setPosition(TRANSFER_POS.UP.val);

            transferTimer.reset();
            transferTimer.startTime();

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>TURRET_LAUNCH_SPEEDS.CLOSE.val/2) {
                magazine[5] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo1.setPosition(TRANSFER_POS.DOWN.val);
//            transfer_servo2.setPosition(TRANSFER_POS.DOWN.val);

            transferTimer.reset();
        }
        transfer_isUp = !transfer_isUp;

    }
    public void transfer_ball(boolean direction){ // true = up, false = down
        if (current_blade_index % 2 == 0) {
            return;
        }
        if(direction){
            transfer_servo1.setPosition(0.4);
//            transfer_servo2.setPosition(TRANSFER_POS.UP.val);

            transferTimer.reset();
            transferTimer.startTime();
            transfer_isUp = true;

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.feedforwardController.targetVelocity>15) {
                magazine[5] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo1.setPosition(TRANSFER_POS.DOWN.val);
//            transfer_servo2.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
            transfer_isUp = false;
        }
    }

    public void rotate_elice(double turns){ // positive is right, negative is left
        if(turns==0) return;
        if (turns>0) {
            pidElice.switchPid(0);
            kF_fornrofballs = Math.abs(kF_fornrofballs);
        }
        else {
            pidElice.switchPid(1);
            kF_fornrofballs = -Math.abs(kF_fornrofballs);

        }
        boolean direction = (turns>0);
        rotate_balls(direction);
        if (turns==(int)turns)
            rotate_balls(direction);
        //pidElice.switchPid(getNumberOfBalls());
        moving_timer.reset();
        moving_timer.startTime();
        last_moving_state = current_moving_state;

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target += turns*one_rotation*2.0;
        pidElice.resetPid();
        pidElice.setTargetPosition(target);
        runPid = true;

        // sa se invarta usurel intake-ul
        intake.toggle(INTAKE_STATES.SLIGHTLY_MOVING);
        slightlyMovingTimer.reset();
        slightlyMovingTimer.startTime();
        slightlyMovingDuration = durationOfOneRotation*turns;
    }

    @Override
    public void update(){
        position = -sorting_encoder.getCurrentPosition();
        blade_rotation = (int)position % (int)full_rotation;
        if(blade_rotation<0)blade_rotation=(int)full_rotation+blade_rotation;
        pidError = target-position;

        if (runPid) {
            double power;
            power = pidElice.update(position) * (14/voltageSensor.getVoltage()) + kF_fornrofballs*getNumberOfBalls();
            //            power = pidElice.update(position) * (14/voltageSensor.getVoltage());
            blade1.setPower(power);
            blade2.setPower(power);
        }
        else if (Math.abs(manualDeviation)>0.2) {
            manualBlade();
        }
        else {
            blade1.setPower(0);
            blade2.setPower(0);
        }

        if (intake.intake_state == INTAKE_STATES.SLIGHTLY_MOVING && (current_moving_state != MOVING_STATES.MOVING || slightlyMovingTimer.seconds() >slightlyMovingDuration)){
            if (intake.fromCollectingToSlightly)
                intake.toggle(INTAKE_STATES.COLLECTING);
            else
                intake.toggle(INTAKE_STATES.STOPPED);
            slightlyMovingTimer.reset();
        }

        if (Math.abs(pidError) > admissible_error*6&& !runPid && Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED) {
            runPid = true;
            pidElice.resetPid();
        }

        if(transfer_isUp && transferTimer.milliseconds()>200){
            transfer_ball(false);
        }

        current_blade_index = pos_to_index((int)position);

        if (current_blade_index != last_blade_index){
            boolean direction_of_rotation;
            if (current_blade_index==6&&last_blade_index==1)
                direction_of_rotation = false;
            else if (current_blade_index==1&&last_blade_index==6)
                direction_of_rotation = true;
            else
                direction_of_rotation =(current_blade_index-last_blade_index>0);

            //rotate_balls(direction_of_rotation);
        }
        last_blade_index = current_blade_index;


        if (current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE&& next_moving_state != MOVING_STATES.NOTHING){
            last_moving_state = current_moving_state;
            current_moving_state = next_moving_state;
            next_moving_state = MOVING_STATES.NOTHING;
            return;
        }
        //MMMMMOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINGGGGGGGGGGGGGGGGGGGGGG
        if (current_moving_state == MOVING_STATES.MOVING){

            if ((Math.abs(pidError)< admissible_error && runPid && !(Math.abs(manualDeviation)>0.2))||moving_timer.milliseconds()>600){
                //if ((position>0?position:-position)>(target>0?target:-target)){
                pidElice.resetPid();
                runPid = false;
                if (last_moving_state==MOVING_STATES.WAITING_INTAKE || last_moving_state==MOVING_STATES.SHOOTING) {
                    current_moving_state = last_moving_state;
                    resetManual();
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
            if (current_blade_index % 2 == 1) {
                if (magazine[1] == COLORS.EMPTY) rotate_elice(0.5);
                else rotate_elice(-0.5);

            }
            COLORS current_color = ColorFunctions.getColor(color_sensor_intake.red(),color_sensor_intake.green(),color_sensor_intake.blue());
            if (current_color != COLORS.EMPTY){
                if(magazine[2] != COLORS.EMPTY){
                    //setNextState(MOVING_STATES.NOTHING);
                    rotate_elice(1);

                    return;
                }

                magazine[2] = current_color;
                if (magazine[6] == COLORS.EMPTY)
                    rotate_elice(1);
                else if (magazine[4] == COLORS.EMPTY)
                    rotate_elice(-1);
            }
        }
      //  SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTIIIIIIIIIIIIIIIIIIIIIINNNNNNNNNNNNNNNNNNNGGGGGGGGG
        if (current_moving_state == MOVING_STATES.SHOOTING){
            if(transfer_isUp || transferTimer.milliseconds()<375)
                return;
            //if(shooting_balls<=0){
            if(shooting_index>3 || shooting_balls==0){
                exit_shooting();
                niggacuplus++;
                return;
            }
            else if (shooting_order[shooting_index]==COLORS.EMPTY && shooting_balls<0){
                exit_shooting();
                niggacuminus++;
                return;
            }
            if (shooting_balls<0) {
                if (magazine[5] == shooting_order[shooting_index] || (!respectMotif && magazine[5] != COLORS.EMPTY) || movedball) {
                    shooting_index++;
                    shooting_balls--;
                    movedball = false;
                    transfer_ball(true);
                } else if (magazine[1] == shooting_order[shooting_index] || (!respectMotif && magazine[1] != COLORS.EMPTY)) {
//                    if (movedball && safe_timer.milliseconds() < 1200) return;
//                    else if (movedball && safe_timer.milliseconds() > 1200) exit_shooting();
                    movedball = true;
                    rotate_elice(-1);
                    return;
                } else if (magazine[3] == shooting_order[shooting_index] || (!respectMotif && magazine[3] != COLORS.EMPTY)) {
//                    if (movedball && safe_timer.milliseconds() < 1200) return;
//                    else if (movedball && safe_timer.milliseconds() > 1200) exit_shooting();
                    movedball = true;

                    rotate_elice(1);
                    return;

                }
                else
                    exit_shooting();
            }

            else {
                if (!shotBall) {
                    transfer_ball(true);
                    shooting_balls--;
                    shotBall = true;
                }
                else {
                    rotate_elice(1);
                    shotBall = false;
                }
            }

        }
//        if (current_moving_state == MOVING_STATES.SHOOTING){
//            if(transfer_isUp || transferTimer.milliseconds()<200) {
//                exit_shooting();
//                return;
//            }
//            //if(shooting_balls<=0){
//            if(shooting_index>3 || shooting_order_powers[shooting_index] == -2){
//                exit_shooting();
//                return;
//            }
//            if (shooting_order_powers[shooting_index]!=0)
//                transfer_ball(true);
//            rotate_elice(shooting_order_powers[shooting_index]);
//            shooting_index++;
//
//        }
        resetManual();
       // telemetryData();
    }
    public void exit_shooting(){
        last_moving_state = MOVING_STATES.NOTHING;
        current_moving_state = MOVING_STATES.NOTHING;
        rotate_elice(-0.5);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);
        //clearMagazine();
    }
    public void sort_balls(){
        int current_green_pos = getGreen();
        double rotations=0;
        // int rotations = motif - current_green_pos;
        int nrofballs = getNumberOfBalls();

        if (nrofballs==0) return;
        if(nrofballs==1){
            for(int i=1;i<=6;i++){
                if(magazine[i] != COLORS.EMPTY) {
                    rotations = (4 - i)/2.0;
                }
            }
        }
        else if (nrofballs==2){
            if (current_blade_index % 2 ==0) {
                if (magazine[2] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY)
                    rotations = -1;
                else if (magazine[4] != COLORS.EMPTY && magazine[2] != COLORS.EMPTY)
                    rotations = 1;
                else if (magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY)
                    rotations = 0;
            }
            else {
                if (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY)
                    rotations = 1.5;
                else if (magazine[1] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)
                    rotations = -0.5;
                else if (magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)
                    rotations = 0.5;
            }
        }
        else if (nrofballs == 3){
            rotations = (motif.val - current_green_pos)/2.0;
        }
        rotations_for_telemetry = rotations;


        rotate_elice(rotations);

    }
    public void shootThreeBalls(){
        shooting_index = 1;
        shooting_balls = 3;
        shotBall = false;
        setNextState(MOVING_STATES.SHOOTING);
    }
    public COLORS[] getShooting_order() {
        int current_shift = shift % 4, current_motif = motif.val;
        COLORS[] order = new COLORS[4];
        current_motif -= shift;
        shooting_balls = -1;
        safe_timer.reset();
        safe_timer.startTime();
        movedball = false;
        if (current_motif <= 0) {
            current_motif = 3 + current_motif;
        }
        safe_timer.reset();
        safe_timer.startTime();
        for (int i = 1; i <= 3; i++) {
            if (current_motif == i) order[i] = COLORS.GREEN;
            else order[i] = COLORS.PURPLE;
            if (i > getNumberOfBalls()) order[i] = COLORS.EMPTY;
        }
        return order;
    }

    public void makeShootingOrder() {
        int current_shift = shift % 4, current_motif = motif.val, index=1;
        COLORS[] order = new COLORS[4];
        COLORS[] magazine_copy = new COLORS[7];

        shooting_index = 1;


        current_motif -= shift;
        if (current_motif <= 0) {
            current_motif = 3 + current_motif;
        }
        for (int i = 1; i <= 3; i++) {
            if (current_motif == i) order[i] = COLORS.GREEN;
            else order[i] = COLORS.PURPLE;
            if (i > getNumberOfBalls()) order[i] = COLORS.EMPTY;
        }
        for(int i=0;i<=6;i++){
            magazine_copy[i] = magazine[i];
        }
        shooting_order_powers[0] = 0;
        if(magazine_copy[5]==COLORS.EMPTY){
            if (magazine_copy[3] == order[1]){
                shooting_order_powers[0] = 1;
                rotate_balls_with_list(magazine_copy,true);
            }
            else {
                shooting_order_powers[0]=-1;
                rotate_balls_with_list(magazine_copy,false);

            }
        }
        else if (respectMotif) {
            if (magazine_copy[5] == COLORS.GREEN && order[1] != COLORS.GREEN) {
                shooting_order_powers[0] = 1;
                rotate_balls_with_list(magazine_copy,true);

            }
            else if (magazine_copy[5] == COLORS.PURPLE && order[1] == COLORS.GREEN){
                shooting_order_powers[0] = -1;
                rotate_balls_with_list(magazine_copy,false);

            }

        }
        for(int i=1;i<=3;i++){
            if (order[index]==COLORS.EMPTY) shooting_order_powers[i] = -2;
            if(magazine_copy[5]==order[index] || (!respectMotif &&magazine_copy[5]!=COLORS.EMPTY)) {
                shooting_order_powers[i] = 0;
                index++;
            }
            if (magazine_copy[1]==order[index] || (!respectMotif && magazine_copy[1]!=COLORS.EMPTY)) {
                shooting_order_powers[i] = -1;
                magazine_copy[1] = COLORS.EMPTY;
                magazine_copy = rotate_balls_with_list(magazine_copy, false);
                index++;
            }
            else if (magazine_copy[3]==order[index] || (!respectMotif && magazine_copy[3]!=COLORS.EMPTY)){
                shooting_order_powers[i] = 1;
                magazine_copy[3] = COLORS.EMPTY;
                index++;
                magazine_copy = rotate_balls_with_list(magazine_copy,true);
            }
            else
                shooting_order_powers[i] = -2;

        }
    }

    public static int pos_to_index(int position){
        if (position<0) position = -position;
        position %= (int)full_rotation;

        int index=0, min_dif=10000000, dif=0;
        for (int i=0;i<=6;i++){
            dif = position-(int)full_rotation/6*i;
            if(dif<0)dif=-dif;

            if (dif<min_dif){
                min_dif = dif;
                index = i;
            }
        }
        index+=2;
        if (index>6) index %=6;
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
    public static COLORS[] rotate_balls_with_list(COLORS[] list,boolean right){
        COLORS aux;
        if (right){
            for(int i=6;i>=2;i--){
                aux = list[i];
                list[i] = list[i-1];
                list[i-1] = aux;
            }
        }
        else {
            for(int i=2;i<=6;i++){
                aux = list[i-1];
                list[i-1] = list[i];
                list[i] = aux;
            }

        }
        return list;

    }
    public boolean isFull(){
        if((magazine[2] != COLORS.EMPTY && magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY) || (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)){
            intake.toggle(INTAKE_STATES.STOPPED);
            if (collectTime==0) {
                collectTime = collectTimer.seconds();
                collectTimer.reset();
            }
            return true;
        } else {
            return false;
        }
    }
    public int getGreen(){
        for(int i=1;i<=6;i++){
            if (magazine[i]==COLORS.GREEN)
                return i;
        }
        return 0;
    }
    public int getNumberOfBalls(){
        int nrofballs=0;
        if (magazine[1]!=COLORS.EMPTY || magazine[3]!=COLORS.EMPTY || magazine[5]!=COLORS.EMPTY){
            magazine[2] = COLORS.EMPTY;
            magazine[4] = COLORS.EMPTY;
            magazine[6] = COLORS.EMPTY;
        }
        else if (magazine[2] != COLORS.EMPTY || magazine[4] != COLORS.EMPTY || magazine[6] != COLORS.EMPTY) {
            magazine[1] = COLORS.EMPTY;
            magazine[3] = COLORS.EMPTY;
            magazine[5] = COLORS.EMPTY;
        }
        for (int i=1;i<=6;i++){
            if (magazine[i]!=COLORS.EMPTY) nrofballs++;
        }
          /* alternative
          int starting_i;

          if (current_blade_index%2==0) starting_i = 2;
          else starting_i = 1;

          for(int i=starting_i;i<=6;i+=2){
              if (magazine[i] != COLORS.EMPTY) {
                  rotations = 4-i;
                  return;
              }
          }
          */
        return nrofballs;

    }
    public void clearMagazine(){
        //        magazine[1] = COLORS.EMPTY;
        //        magazine[2] = COLORS.GREEN;
        //        magazine[3] = COLORS.EMPTY;
        //        magazine[4] = COLORS.PURPLE;
        //        magazine[5] = COLORS.EMPTY;
        //        magazine[6] = COLORS.PURPLE;
        for(int i=1;i<=6;i++){
            magazine[i] = COLORS.EMPTY;
        }
    }
    public void fillMagazine(){
        magazine[1] = COLORS.EMPTY;
        magazine[2] = COLORS.GREEN;
        magazine[3] = COLORS.EMPTY;
        magazine[4] = COLORS.PURPLE;
        magazine[5] = COLORS.EMPTY;
        magazine[6] = COLORS.PURPLE;
    }



    public void telemetryData(){
        telemetry.addData("Absolute bladep pos: ",position);
        telemetry.addData("Moded blade pos: ",blade_rotation);
        telemetry.addData("STATEEEE", current_moving_state.val);
        telemetry.addData("Number of balls: ",getNumberOfBalls());
        telemetry.addData("MOTIFFFF", motif);

        telemetry.addData("Current_blade_index", current_blade_index);
        telemetry.addData("Pos_to_index",pos_to_index((int)position));
        telemetry.addData("blade_rotation % one_rotation",blade_rotation%one_rotation);
        telemetry.addData("Target: ",target);
        telemetry.addData("Pid error",pidError);
        telemetry.addData("Timer: ",transferTimer.milliseconds());
        telemetry.addData("runPid: ",runPid);
        telemetry.addData("Which pid: ", pidElice.whichPid());
        telemetry.addData("Collecting time: ",collectTime);
        telemetry.addData("Rotations for sorting: ",rotations_for_telemetry);

        telemetry.addData("1: ", magazine[1].val);
        telemetry.addData("2: ", magazine[2].val);
        telemetry.addData("3: ", magazine[3].val);
        telemetry.addData("4: ", magazine[4].val);
        telemetry.addData("5: ", magazine[5].val);
        telemetry.addData("6: ", magazine[6].val);

        telemetry.addData("shooting order 1: ",shooting_order[1]);
        telemetry.addData("shooting order 2: ",shooting_order[2]);
        telemetry.addData("shooting order 3: ",shooting_order[3]);


        telemetry.addData("s-a iesit din primu if", niggacuplus);
        telemetry.addData("s-a iesit din al doilea if  ", niggacuminus);


        telemetry.addLine("--------------------------------------------------"); // separate the mechanisms to make the text easier to read


    }


}