  package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.MultiplePIDF;
import org.firstinspires.ftc.teamcode.lib.tests.ColorFunctions;


//     (. Y .)
//       ).(
//      \ Y /
//       | |



@Config
public class Sorting implements  Updateable{
//---------MOTORS--------------------
    public Servo transfer_servo;
    public DcMotorEx blade;
    public ColorSensor color_sensor_human;
    public ColorSensor color_sensor_intake;
    private final Intake intake;
    //public static PIDF pidElice = new PIDF(0.000163d,0.0000062d,0.00000091d);

    public static MultiplePIDF pidElice = new MultiplePIDF(0.000163d,0.0000062d,0.0000009d);

    private final Telemetry telemetry;


//----------POSITIONS-------------
    public TRANSFER_POS transfer_pos = TRANSFER_POS.DOWN;
    public static final double full_rotation = 384.5;
    public static final double one_rotation = full_rotation/6.0;
    public double position=0; // increases or decreses forever;
    public int blade_rotation=0; // betwen [0, full_rotation] equal to position % full_rotation
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
    public static double shooting_const = 16;
    public boolean isShootingPosition = false;
    public double collectTime = 0;
    private double rotations_for_telemetry;


//----------STATES-----------------
    public static COLORS[] magazine = new COLORS[7];// 2 - intake || 4 - aruncare  || 6 - rezerva   1 - intrare human
    public int motif=6; // 4/6/2
    public MOVING_STATES current_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES next_moving_state = MOVING_STATES.NOTHING;
    public MOVING_STATES last_moving_state = MOVING_STATES.NOTHING;



    public Sorting(HardwareMap hwmap, Telemetry telemetry, Intake intake){
        blade = hwmap.get(DcMotorEx.class, HardwareConfig.sorting);
        transfer_servo = hwmap.get(Servo.class,HardwareConfig.transfer);
        color_sensor_human = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_human);
        color_sensor_intake = hwmap.get(ColorSensor.class, HardwareConfig.color_sensor_intake);
        this.intake = intake;
        this.telemetry = telemetry;


        //pidElice = new PIDF(0.000163d,0.0000062d,0.0000009d)// 0 bile
        pidElice.addPidCoefficients(0.000155d,0.0000056d,0.0000005d);// 1 bile
        pidElice.addPidCoefficients(0.00015d,0.0000042,0.00000091d);// 2 bile
        pidElice.addPidCoefficients(0.00012d,0.000002d,0.000000301d);// 3 bile
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

    public enum TRANSFER_POS{
        UP(0.85),DOWN(0);
        final double val;
        TRANSFER_POS(double val) {
            this.val = val;
        }
    }
    public enum MOVING_STATES {
        WAITING_HUMAN_PLAYER("waiting_human_player"), WAITING_INTAKE("waiting_intake"), NOTHING("nothing"), MOVING("moving") , SHOOTING("shooting");
        final String val;
        MOVING_STATES(String val){this.val = val;}
    }

    public enum COLORS{
        GREEN("green"), PURPLE("parpal"), EMPTY("empty");
        public final String val;
        COLORS(String val) {
            this.val = val;
        }
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

                if (current_blade_index % 2 == 1) {
                    current_moving_state = state;
                    rotate_elice(0.5);
                    return;
                }
            } else if (state == MOVING_STATES.WAITING_HUMAN_PLAYER) {
                current_moving_state = state;
                if (current_blade_index % 2 == 0) {
                    if (magazine[6] == COLORS.EMPTY)
                        rotate_elice(0.5);
                    else if (magazine[2] == COLORS.EMPTY)
                        rotate_elice(-0.5);
                    else
                        rotate_elice(1.5);
                    return;
                }
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
        intake.toggle(Intake.INTAKE_STATES.STOPPED);
        Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.STOPPED);

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

            if(Turret.turret_launcher_state != Turret.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                magazine[4] = COLORS.EMPTY;
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

            if(Turret.turret_launcher_state != Turret.TURRET_LAUNCH_SPEEDS.STOPPED || Turret.velocityPID.targetVelocity>500) {
                magazine[4] = COLORS.EMPTY;
            }
        }
        else{
            transfer_servo.setPosition(TRANSFER_POS.DOWN.val);
            transferTimer.reset();
            transfer_isUp = false;
        }
    }

    public void rotate_elice(double turns){ // positive is right, negative is left
        /* DOAR UN PIDDD
        if (getNumberOfBalls()==0 && (int)turns!=turns)
            pidElice.switchPid(4);
        else
            pidElice.switchPid(getNumberOfBalls());
        pidElice.switchPid(getNumberOfBalls());

         */
        isSlightlyMovingActivatable = true;


        last_moving_state = current_moving_state;

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target = blade_target + turns*one_rotation*2.0;
        blade_target += turns*one_rotation*2;
        pidElice.resetPid();
        pidElice.setTargetPosition(target);
        runPid = true;
        isShootingPosition = false;
    }
    public void rotate_elice(double turns, boolean shooting){ // positive is right, negative is left
        /* ONE PIDDDD
        if (turns==0 && shooting)
            pidElice.switchPid(5+getNumberOfBalls());
        else
            pidElice.switchPid(getNumberOfBalls());

         */
        isSlightlyMovingActivatable = true;


        last_moving_state = current_moving_state;
        double delta_shooting = 0;
        if (shooting){
            delta_shooting = shooting_const;
            if(turns<0)
                delta_shooting += delta_shooting/2.3;
            isShootingPosition = true;
        }

        current_moving_state = MOVING_STATES.MOVING;
        turns = turns % 4; //o sa ai valori cuprinse intre -3 si 3
        //target = (int)(position / full_rotation) * full_rotation + full_rotation/6 * pos_to_index((int)position) + one_rotation*turns*2;
        target = blade_target + turns*one_rotation*2.0 + delta_shooting;
        blade_target += turns*one_rotation*2;
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
            double nigger;
            nigger = pidElice.update(position);
            blade.setPower(nigger);
        }
        else blade.setPower(0);

        if (intake.intake_state != Intake.INTAKE_STATES.SLIGHTLY_MOVING && isSlightlyMovingActivatable) {
            if (current_moving_state == MOVING_STATES.MOVING) {
                intake.toggle(Intake.INTAKE_STATES.SLIGHTLY_MOVING);
                slightlyMovingTimer.reset();
                slightlyMovingTimer.startTime();
                isSlightlyMovingActivatable = false;
            }
        }
        else if (intake.intake_state == Intake.INTAKE_STATES.SLIGHTLY_MOVING && (current_moving_state != MOVING_STATES.MOVING || slightlyMovingTimer.seconds() >0.7)){
            if (intake.fromCollectingToSlightly)
                intake.toggle(Intake.INTAKE_STATES.COLLECTING);
            else
                intake.toggle(Intake.INTAKE_STATES.STOPPED);
            slightlyMovingTimer.reset();
        }

         if (Math.abs(pidError) > admissible_error*5 && !runPid && Turret.turret_launcher_state == Turret.TURRET_LAUNCH_SPEEDS.STOPPED) {
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

                rotate_balls(direction_of_rotation);
        }
        last_blade_index = current_blade_index;


        if ((current_moving_state == MOVING_STATES.NOTHING || current_moving_state == MOVING_STATES.WAITING_INTAKE || current_moving_state == MOVING_STATES.WAITING_HUMAN_PLAYER)&& next_moving_state != MOVING_STATES.NOTHING){
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
                if(magazine[2] != COLORS.EMPTY){
                    setNextState(MOVING_STATES.NOTHING);
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
                magazine[1] = current_color;
                if (magazine[5] == COLORS.EMPTY)
                    rotate_elice(1);
                else if (magazine[3] == COLORS.EMPTY)
                    rotate_elice(-1);

            }
        }
//SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTIIIIIIIIIIIIIIIIIIIIIINNNNNNNNNNNNNNNNNNNGGGGGGGGG
        if (current_moving_state == MOVING_STATES.SHOOTING){
            if(shooting_balls<=0){
                last_moving_state = MOVING_STATES.NOTHING;
                current_moving_state = MOVING_STATES.NOTHING;
                rotate_elice(1);
                Turret.setTarget_rotation(Turret.TURRET_LAUNCH_SPEEDS.STOPPED);
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
            rotations = (motif - current_green_pos)/2.0;
        }
        rotations_for_telemetry = rotations;
        if(rotations==0) {
            rotate_elice(0,true);
            return;
        }

        rotate_elice(rotations,true);

    }
    public void shoot(){
        if(Math.abs(pidError)<admissible_error*3.2){
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
        position %= full_rotation;

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
    public boolean isFull(){
        if((magazine[2] != COLORS.EMPTY && magazine[4] != COLORS.EMPTY && magazine[6] != COLORS.EMPTY) || (magazine[1] != COLORS.EMPTY && magazine[3] != COLORS.EMPTY && magazine[5] != COLORS.EMPTY)){
            intake.toggle(Intake.INTAKE_STATES.STOPPED);
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
        telemetry.addData("4: ", magazine[4].val);
        telemetry.addData("5: ", magazine[5].val);
        telemetry.addData("6: ", magazine[6].val);



    }


}
