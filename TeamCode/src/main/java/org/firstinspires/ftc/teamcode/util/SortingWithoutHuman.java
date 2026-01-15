package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants_Enums.MOVING_STATES;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.COLORS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TRANSFER_POS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.TURRET_LAUNCH_SPEEDS;
import org.firstinspires.ftc.teamcode.util.Constants_Enums.INTAKE_STATES;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.MultiplePIDF;
import org.firstinspires.ftc.teamcode.lib.PIDF;
import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;


//     (. Y .)
//       ).(
//      \ Y /
//       | |



@Config
public class SortingWithoutHuman implements  Updateable{
    //---------MOTORS--------------------
    public Servo transfer_servo;
    public DcMotorEx blade;
    public ColorSensor color_sensor_human;
    public ColorSensor color_sensor_intake;
    private final Intake intake;
    private final VoltageSensor voltageSensor;
    //public static PIDF pidElice = new PIDF(0.000163d,0.0000062d,0.00000091d);
    public static MultiplePIDF pidElice = new MultiplePIDF(0.00198,0.0002,2e-5d);

    private final Telemetry telemetry;


    //----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final double full_rotation = 384.5;
    public static final double one_rotation = full_rotation/3.0;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // between [0, full_rotation] equal to position % full_rotation
    public int current_blade_index=0;
    public int last_blade_index=0;
    public double target;
    private double blade_target=0;
    public double pidError;
    private final ElapsedTime transferTimer, collectTimer, slightlyMovingTimer;
    private boolean transfer_isUp = false;
    private int shooting_balls = 0;
    private static final int admissible_error=7;
    private boolean runPid=false;
    private boolean isManually = false;
    private boolean isSlightlyMovingActivatable = false;
    public static double shooting_const = 23;
    public static double kF_fornrofballs=0;
    public boolean isShootingPosition = false;
    public double collectTime = 0;
    private double rotations_for_telemetry;

    //momentul de frecare e momentu care se opune momentului motorului (care misca elicea)
//
//----------STATES-----------------
    public static COLORS[] magazine = new COLORS[4];// 1 intake | 2 tureta | 3 storage
    public int motif=1;
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public SortingWithoutHuman(HardwareMap hwmap, Telemetry telemetry, Intake intake, VoltageSensor voltageSensor){
        blade = hwmap.get(DcMotorEx.class, HardwareConfig.sorting);
        transfer_servo = hwmap.get(Servo.class,HardwareConfig.transfer);
        color_sensor_human = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_human);
        color_sensor_intake = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_intake);
        this.voltageSensor = voltageSensor;
        this.intake = intake;
        this.telemetry = telemetry;


        //pidElice = new PIDF(0.000163d,0.0000062d,0.0000009d)// 0 bile
        pidElice.addPidCoefficients(0.00198,0.0002,2e-5d);// 1 bile
        pidElice.addPidCoefficients(0.00198,0.0002,2e-5d);// 2 bile
        pidElice.addPidCoefficients(0.00198,0.0002,2e-5d);// 3 bile

        pidElice.addPidCoefficients(0.00022d,0.000001d,0.0000004d);// (4) half rotation

        /// ///////////////
        pidElice.addPidCoefficients(0.00045d,0.0000245d,0.0000005d);// 5) quarter rotation
        pidElice.addPidCoefficients(0.00052d,0.000007d,0.0000005d,0.00002);// 6) 1 ball
        pidElice.addPidCoefficients(0.00043d,0.0000008d,0,0.00003); // 7 2 ball
        pidElice.addPidCoefficients(0.00042d, 0.00000026d,0.00000001d, 0.000015d); // 8 3 ball


        transferTimer = new ElapsedTime();
        collectTimer = new ElapsedTime();
        slightlyMovingTimer = new ElapsedTime();
        transfer_servo.setDirection(Servo.Direction.REVERSE);

        transfer_servo.setPosition(transfer_pos.val);

        blade.setDirection(DcMotorSimple.Direction.REVERSE);
        blade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        blade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blade.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);


        pidElice.resetPid();
        clearMagazine();

    }



    public void setNextState(MOVING_STATES state){
//        if(state == MOVING_STATES.SHOOTING && Turret.turret_launcher_state == Turret.TURRET_LAUNCH_SPEEDS.STOPPED)
//            return;
        if (current_moving_state == MOVING_STATES.MOVING) {
            next_moving_state = state;
        }

        else {
            if (state == MOVING_STATES.WAITING_INTAKE) {
                collectTimer.reset();
                collectTimer.startTime();
                collectTime = 0;
            }

            current_moving_state = state;
        }



        pidElice.resetPid();
    }
    public void resetStates(){
        current_moving_state = MOVING_STATES.NOTHING;
        next_moving_state = MOVING_STATES.NOTHING;
        pidElice.resetPid();
    }
    public void manualBlade(double delta){
        pidElice.setTargetWithoutResetting(delta);
        target+=delta;
        runPid = true;
        isManually = true;
    }
    public void resetEverything(){
        resetStates();
        pidElice.setTargetPosition(0);
        pidElice.resetPid();
        last_moving_state = MOVING_STATES.NOTHING;
        last_blade_index = 0;
        current_blade_index = 0;
        isManually = false;
        runPid = false;
        transfer_ball(false);
        transfer_isUp = false;
        clearMagazine();
        pidError = 0;

        position = 0;
        blade_rotation = 0;
        blade_target = 0;
        target = 0;
        shooting_balls = 0;
        target = 0;
        pidError = 0;
        intake.toggle(INTAKE_STATES.STOPPED);
        Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);

        blade.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        blade.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blade.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //encoder_elice.setDirection(DcMotorSimple.Direction.REVERSE);



    }
    public void transfer_ball(){ //toggle
        if(!transfer_isUp){
            transfer_servo.setPosition(TRANSFER_POS.UP.val);

            transferTimer.reset();
            transferTimer.startTime();

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                magazine[2] = COLORS.EMPTY;
            }
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

            if(Turret.turret_launcher_state != TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                magazine[2] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
            transfer_isUp = false;
        }
    }

    public void rotate_elice(int turns){ // positive is right, negative is left
        if (turns==2) turns = -1;
        else if (turns==-2) turns = 1;

        pidElice.switchPid(0);
        //pidElice.switchPid(getNumberOfBalls());
        isSlightlyMovingActivatable = true;

        last_moving_state = current_moving_state;

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target = blade_target + turns*one_rotation;
        blade_target += turns*one_rotation;
        pidElice.resetPid();
        pidElice.setTargetPosition(target);
        runPid = true;
        isShootingPosition = false;
    }
    public void rotate_elice(int turns, boolean shooting){ // positive is right, negative is left
        /* ONE PIDDDD
        if (turns==0 && shooting)
            pidElice.switchPid(5+getNumberOfBalls());
        else
            pidElice.switchPid(getNumberOfBalls());

         */
        if (turns==2) turns = -1;
        else if (turns==-2) turns = 1;

        isSlightlyMovingActivatable = true;


        last_moving_state = current_moving_state;
        double delta_shooting = 0;
        if (shooting){
            delta_shooting = shooting_const;
//            if(turns<0)
//                delta_shooting += delta_shooting/2.3;
            isShootingPosition = true;
        }

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target = blade_target + turns*one_rotation + delta_shooting;
        blade_target += turns*one_rotation;
        pidElice.resetPid();
        pidElice.setTargetPosition(target);
        runPid = true;
    }

    @Override
    public void update(){
        position = blade.getCurrentPosition();
        blade_rotation = (int)position % (int)full_rotation;
        if(blade_rotation<0)blade_rotation=(int)full_rotation+blade_rotation;
        pidError = target-position;

        if (runPid || isManually) {
            double power;
            power = pidElice.update(position) * (14/voltageSensor.getVoltage()) + kF_fornrofballs*getNumberOfBalls();
//            power = pidElice.update(position) * (14/voltageSensor.getVoltage());
            blade.setPower(0);
        }
        else blade.setPower(0);

        if (intake.intake_state != INTAKE_STATES.SLIGHTLY_MOVING && isSlightlyMovingActivatable) {
            if (current_moving_state == MOVING_STATES.MOVING) {
                intake.toggle(INTAKE_STATES.SLIGHTLY_MOVING);
                slightlyMovingTimer.reset();
                slightlyMovingTimer.startTime();
                isSlightlyMovingActivatable = false;
            }
        }
        else if (intake.intake_state == INTAKE_STATES.SLIGHTLY_MOVING && (current_moving_state != MOVING_STATES.MOVING || slightlyMovingTimer.seconds() >0.7)){
            if (intake.fromCollectingToSlightly)
                intake.toggle(INTAKE_STATES.COLLECTING);
            else
                intake.toggle(INTAKE_STATES.STOPPED);
            slightlyMovingTimer.reset();
        }

        if (Math.abs(pidError) > admissible_error*5 && !runPid && Turret.turret_launcher_state == TURRET_LAUNCH_SPEEDS.STOPPED) {
            runPid = true;
            pidElice.resetPid();
        }

        if(transfer_isUp && transferTimer.milliseconds()>200){
            transfer_ball(false);
        }

        current_blade_index = pos_to_index((int)position);

        if (current_blade_index != last_blade_index){
            boolean direction_of_rotation;
            if (current_blade_index==3&&last_blade_index==1)
                direction_of_rotation = false;
            else if (current_blade_index==1&&last_blade_index==3)
                direction_of_rotation = true;
            else
                direction_of_rotation =(current_blade_index-last_blade_index>0);

            rotate_balls(direction_of_rotation);
        }
        last_blade_index = current_blade_index;


        if ((current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE)&& next_moving_state != MOVING_STATES.NOTHING){
            last_moving_state = current_moving_state;
            current_moving_state = next_moving_state;
            next_moving_state = MOVING_STATES.NOTHING;
            return;
        }
//MMMMMOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIINGGGGGGGGGGGGGGGGGGGGGG
        if (current_moving_state == MOVING_STATES.MOVING){
//            if (next_moving_state != MOVING_STATES.NOTHING){
//                last_moving_state = current_moving_state;
//                current_moving_state = next_moving_state;
//                next_moving_state = MOVING_STATES.NOTHING;
//                return;
//            }

            if ((pidError>0?pidError:-pidError) < admissible_error && runPid && !isManually){
                //if ((position>0?position:-position)>(target>0?target:-target)){
                pidElice.resetPid();
                runPid = false;
                //rotate_elice(0) posibil sa ajute la un moment dat;
                if (last_moving_state==MOVING_STATES.WAITING_INTAKE || last_moving_state==MOVING_STATES.SHOOTING) {
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
                if(magazine[1] != COLORS.EMPTY){
                    setNextState(MOVING_STATES.NOTHING);
                    rotate_elice(1);

                    return;
                }

                magazine[1] = current_color;
                if (magazine[3] == COLORS.EMPTY)
                    rotate_elice(1);
                else
                    rotate_elice(-1);
            }
        }

//SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTIIIIIIIIIIIIIIIIIIIIIINNNNNNNNNNNNNNNNNNNGGGGGGGGG
        if (current_moving_state == MOVING_STATES.SHOOTING){
            if(shooting_balls<=0){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                rotate_elice(1);
                Turret.setTarget_rotation(TURRET_LAUNCH_SPEEDS.STOPPED);
                clearMagazine();
                return;
            }
            shooting_balls--;
            transfer_ball(true);
            rotate_elice(-1,true);

        }
        isManually = false;
    }
    public void sort_balls(){
        int current_green_pos = getGreen();
        int rotations=0;
        // int rotations = motif - current_green_pos;
        int nrofballs = getNumberOfBalls();

        if (nrofballs==0) return;
        if(nrofballs==1){
            for(int i=1;i<=3;i++){
                if(magazine[i] != COLORS.EMPTY) {
                    rotations = 2 - i;
                    continue;
                }
            }
        }
        else if (nrofballs==2){
            for(int i=1;i<=3;i++){
                if(magazine[i] == COLORS.EMPTY){
                    rotations = 1-i;
                }
            }
        }
        else if (nrofballs == 3){
            rotations = motif - current_green_pos;
        }
        rotations_for_telemetry = rotations;
        if(rotations==0) {
            rotate_elice(0,true);
            return;
        }

        rotate_elice(rotations,true);

    }
    public void shoot(){
        if(Math.abs(pidError)<admissible_error*1.1){ // se poate schimba valorea
            current_moving_state = MOVING_STATES.NOTHING;
            last_moving_state = MOVING_STATES.NOTHING;
        }
        sort_balls();
        shooting_balls = getNumberOfBalls();
        if (shooting_balls == 0)
            return;
        setNextState(MOVING_STATES.SHOOTING);
    }

    public static int pos_to_index(int position){
        if (position<0) position = -position;
        position %= (int)full_rotation;

        int index=0, min_dif=10000000, dif=0;
        for (int i=0;i<=3;i++){
            dif = position-(int)full_rotation/3*i;
            if(dif<0)dif=-dif;

            if (dif<min_dif){
                min_dif = dif;
                index = i;
            }
        }
        index+=1;
        if (index>3) index %=3;
        return index;
    }

    public static void rotate_balls(boolean right){
        COLORS aux;
        if (right){
//            for(int i=3;i>=2;i--){
//                aux = magazine[i];
//                magazine[i] = magazine[i-1];
//                magazine[i-1] = aux;
//            }
            aux = magazine[3];
            magazine[3] = magazine[2];
            magazine[2] = aux;

            // 1 2 3 -> 1 3 2

            magazine[2] = magazine[1];
            magazine[1] = aux;

            // 3 1 2
         }
        else {
//            for(int i=2;i<=3;i++){
//                aux = magazine[i-1];
//                magazine[i-1] = magazine[i];
//                magazine[i] = aux;
//            }
            aux = magazine[1];
            magazine[1] = magazine[2];
            magazine[2] = aux;

            // 1 2 3 -> 1 3 2

            magazine[2] = magazine[3];
            magazine[3] = aux;

            // 3 1 2

        }

    }
    public boolean isFull(){
        if(magazine[1] != COLORS.EMPTY && magazine[2] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY){
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
    public static int getGreen(){
        for(int i=1;i<=3;i++){
            if (magazine[i]==COLORS.GREEN)
                return i;
        }
        return 0;
    }
    public int getNumberOfBalls(){
        int nrofballs=0;

        for (int i=1;i<=3;i++){
            if (magazine[i]!=COLORS.EMPTY) nrofballs++;
        }
        return nrofballs;

    }
    public void clearMagazine(){
        for(int i=1;i<=3;i++){
            magazine[i] = COLORS.EMPTY;
        }
    }
    public void fillMagazine(){
        magazine[1] = COLORS.PURPLE;
        magazine[2] = COLORS.GREEN;
        magazine[3] = COLORS.PURPLE;
    }
    private int motifToInt(int motif){
        if (motif==1)
            return 2;
        else if (motif==2)
            return 3;
        return 1;
    }



    public  void telemetryData(){
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

    }


}
