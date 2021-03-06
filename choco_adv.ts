//巧克力编程盒竞技版
//% color="#f97c04" weight=25 icon="\uf1b9" block="巧克力编程盒"
namespace ChocoRobot {

    // let liner_buf:Buffer;
    // let color_buf:Buffer;
    // let odom_buf:Buffer;
    let turn_off = false
    let rightspeed = 0;     /** 左右轮速度，单位为cm/s */
    let leftspeed = 0;
    let base_controller: Move_base
    let moving=0;
    const update_rate = 25;
    const MAX_SPEED = 40; /** 最大速度设为40cm/s */
    const MAX_ANGULAR_SPEED =1.2 /** 最大角速度 1.2rad/s */
    const ticks_per_meter = 1710;
    const wheel_track = 21.2
    const pi = 3.1415926
    export class Pose {
        x: number
        y: number
        theta: number

        Pose() {
            this.x = 0
            this.y = 0
            this.theta = 0
        }
    }
    export class Velocity {
        linear_x: number
        linear_y: number
        angular_z: number

        Velocity() {
            this.linear_x = 0
            this.linear_y = 0
            this.angular_z = 0
        }
    }
    export class Move_base {
        goal_pose: Pose
        my_pose: Pose
        vel_cmd: Velocity
        distance_tolerance: number
        angular_tolerance: number
        wheel_track: number      /** 单位cm */
        left_ticks: number
        right_ticks: number
        Move_base() {
            this.distance_tolerance = 8
            this.angular_tolerance = 0.1
            this.left_ticks = 0
            this.right_ticks = 0
        }
        private pow(num: number): number {
            return num * num
        }

        /** get current pose */
        private update_pose() {
            let cmd = pins.createBuffer(5);
            cmd[0] = 0xc3
            cmd[1] = 0
            cmd[2] = 0
            cmd[3] = 0
            cmd[4] = 0
            pins.i2cWriteBuffer(0x00, cmd);
            let odom_buf = pins.createBuffer(6)
            odom_buf = pins.i2cReadBuffer(0x00, 6, false);      /** 下位机里程计以米为单位，发送时乘了1000转化成整数 */
            this.my_pose.x = (odom_buf[1]*256 + odom_buf[0]-32767) /50.0   /** 转换到cm */
            this.my_pose.y = (odom_buf[3]*256 + odom_buf[2]-32767) /50.0
            this.my_pose.theta = (odom_buf[5]*256 + odom_buf[4]-32767) /5000.0/** 转换到弧度 */
        }

        private update_ticks() {
            let cmd = pins.createBuffer(5);
            cmd[0] = 0xc5
            cmd[1] = 0
            cmd[2] = 0
            cmd[3] = 0
            cmd[4] = 0
            pins.i2cWriteBuffer(0x00, cmd);
            let ticks_buf = pins.createBuffer(2)
            ticks_buf = pins.i2cReadBuffer(0x00, 2, false);
            this.left_ticks = ticks_buf[0]
            this.right_ticks = ticks_buf[1]
        }

        /** Euclidean distance between current pose and the goal. */
        euclidean_distance():number{
            return Math.sqrt(this.pow(this.goal_pose.x - this.my_pose.x) +
                this.pow(this.goal_pose.y - this.my_pose.y))
        }

        /** calculate liner velocity */
        liner_vel(distance:number):number{
            return Math.min(1.5*distance,10)/10.0*MAX_SPEED
        }

        /** calculate steering_angle */
        steering_angle():number{
            return Math.atan2(this.goal_pose.y - this.my_pose.y,
                this.goal_pose.x - this.my_pose.x)-this.my_pose.theta
        }

        /** calculate angular velocity */
        angular_vel():number{
            return 5*this.steering_angle()*  MAX_ANGULAR_SPEED
        }

        /** 逆运动学求解 */
        cal_velcmd(vel: Velocity) {
            if (vel.linear_x == 0)
            /** turn in place */ {
                rightspeed = vel.angular_z * wheel_track / 2
                leftspeed = -rightspeed
            }
            else if (vel.angular_z == 0)
            /** pure forward/backward motion */ {
                leftspeed = rightspeed = vel.linear_x
            }
            else {
                leftspeed = vel.linear_x - vel.angular_z * wheel_track / 2
                rightspeed = vel.linear_x + vel.angular_z * wheel_track / 2
            }
        }
        /** x,y坐标单位为cm，theta为弧度 */
        move2goal(speed: number, x: number, y: number, theta: number) {
            this.goal_pose = new Pose();
            this.my_pose = new Pose();
            this.vel_cmd = new Velocity();
            this.goal_pose.x = x;
            this.goal_pose.y = y;
            this.goal_pose.theta = theta*pi/180;
            this.distance_tolerance=5.0
            moving = 1
            /** while not arrive,keep going, */
            while (1) {
                this.update_pose();
                let distance = this.euclidean_distance()
                if(distance <= this.distance_tolerance)
                    break
                this.vel_cmd.linear_x = this.liner_vel(distance) * speed / 100
                this.vel_cmd.angular_z = this.angular_vel() * speed / 100
                this.vel_cmd.linear_y = 0
                this.cal_velcmd(this.vel_cmd);
                basic.pause(1000 / update_rate);
            }
            leftspeed = rightspeed = 255;/** 制动 */
            basic.pause(300);
            moving = 0
            /** stop and turn in place until the right pose */
            if(Math.abs(180*(this.goal_pose.theta-this.my_pose.theta))/pi>3)
                this.turn_in_place(speed,180*(this.goal_pose.theta-this.my_pose.theta)/pi)

        }
        /** speed 0~100,distance单位为cm，正数前进，负数后退 */
        go_line(speed: number, distance: number) {
            let target_ticks = distance * ticks_per_meter / 100 /** 需要走过的目标脉冲数=距离（cm）*每米脉冲数÷100 */
            let sum_ticks = 0                /** 已经走过的脉冲数，取左右轮的平均 */
            let reverse = 1;
            if (target_ticks < 0)
            {
                reverse = -1
                target_ticks = -target_ticks
            }
            moving = 1    
            while (1) {
                this.update_ticks();
                let last_ticks = (this.left_ticks + this.right_ticks) / 2
                sum_ticks += last_ticks
                let ticks_left=target_ticks-sum_ticks
                if (ticks_left <= last_ticks)
                    break;
                leftspeed = reverse * Math.min(ticks_left+50,180)/180.0 * MAX_SPEED* speed / 100 * (this.right_ticks+1)/(this.left_ticks+1);
                rightspeed = reverse * Math.min(ticks_left+50,180)/180.0 * MAX_SPEED* speed / 100;
                basic.pause(1000 / update_rate)
            }
            leftspeed = rightspeed = 255;/** 制动 */
            basic.pause(100)
            moving = 0
        }
        /** speed 0~100 ,angle单位为角度*/
        turn_in_place(speed: number, angle: number) {
            //let target_ticks = angle *3.14159 /180.0 * ticks_per_meter /10 * wheel_track  /** 需要走过的目标脉冲数 */
            let target_ticks = angle * ticks_per_meter /735
            let sum_ticks = 0                /** 已经走过的脉冲数，取左右轮的平均 */
            let reverse = 1;
            if (target_ticks < 0)
            {
                reverse = -1
                target_ticks = -target_ticks
            }
            moving = 1    
            while (1) {
                this.update_ticks();
                let last_ticks = (this.left_ticks + this.right_ticks) / 2
                sum_ticks += last_ticks
                let ticks_left=target_ticks-sum_ticks
                if (ticks_left <= last_ticks)
                    break;
                leftspeed = -reverse * Math.min(ticks_left+50,180)/180.0 * speed *MAX_SPEED/ 100 * (this.right_ticks+1)/(this.left_ticks+1);
                rightspeed = reverse * Math.min(ticks_left+50,180)/180.0 * speed *MAX_SPEED/ 100;
                basic.pause(1000 / update_rate)
            }
            leftspeed = rightspeed = 255;/** 制动 */
            basic.pause(100)
            moving = 0
        }

    }
    export enum enServo {
        //blockId=servo_s1 block="接口1"
        S1 = 1,
        //blockId=servo_s2 block="接口2"
        S2,
        //blockId=servo_s3 block="接口3"
        S3,
        //blockId=servo_s4 block="接口4"
        S4
    }

    export enum CarMove {
        //% blockId="Car_Run" block="前进"
        Car_Run = 1,
        //% blockId="Car_Back" block="后退"
        Car_Back = 2
    }

    export enum CarTurn {
        //% blockId="Car_SpinLeft" block="原地左转"
        Car_SpinLeft = 1,
        //% blockId="Car_SpinRight" block="原地右转"
        Car_SpinRight = 2
    }

    /** 初始化底盘控制器 */
    //% blockId=init_basecontroller block="重置底盘控制器" color="#d43717"
    //% weight=98
    //% parts="move_base"
    export function movebase_init() {
        base_controller = new Move_base()
    }

    //% blockId=Choco_init block="初始化机器人" color="#d43717"
    //% weight=99 
    export function Choco_init() {
        let cmd = pins.createBuffer(5);
        cmd[0] = 0xb0;
        cmd[1] = 0
        cmd[2] = 0
        cmd[3] = 0
        cmd[4] = 0
        pins.i2cWriteBuffer(0x00, cmd);
        let ret = pins.createBuffer(2)
        ret = pins.i2cReadBuffer(0x00, 2, false);
        if (ret[0] == 0xa0 && ret[1] == 0xa0)
            move(0, 0)
        // else 
        //     control.reset()
        gripper_control(1);
        pins.setPull(DigitalPin.P1, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P2, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P8, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P12, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P14, PinPullMode.PullUp)
        pins.setPull(DigitalPin.P15, PinPullMode.PullUp)
        music.playTone(262, music.beat(BeatFraction.Quarter))
        music.playTone(330, music.beat(BeatFraction.Quarter))
        music.playTone(392, music.beat(BeatFraction.Half))
        music.playTone(523, music.beat(BeatFraction.Half))
        movebase_init();
        control.inBackground(() => {    //以update_rate为频率更新两轮速度
            while (1) {
                if(moving)
                {
                    cmd[0] = 0xc0;
                    if(leftspeed==0&&rightspeed==0)
                    {
                        cmd[1] = cmd [2] = 100;
                        moving=0
                    }    
                    else if(leftspeed==255&&rightspeed==255)
                        cmd[1] = cmd [2] =255;
                    else
                    {
                        cmd[1] = Math.round(Math.max(Math.min(leftspeed, MAX_SPEED),-MAX_SPEED) * ticks_per_meter / 100 / update_rate) + 100;    /** 换算到每个控制周期内的tick数 */
                        cmd[2] = Math.round(Math.max(Math.min(rightspeed, MAX_SPEED),-MAX_SPEED) * ticks_per_meter / 100 / update_rate) + 100;   /** +100避免负数 */
                    }
                    cmd[3] = 0
                    cmd[4] = 0
                    pins.i2cWriteBuffer(0x00, cmd);
                }
                basic.pause(1000 / update_rate);
            }
        })
    }
    /** 手动设定左右轮速度 */
    //% blockId=Choco_move block="设置电机速度 左轮 %lspeed |右轮 %rspeed" weight=90
    //% lspeed.min=-100 lspeed.max=100
    //% rspeed.min=-100 rspeed.max=100
    //% group="移动控制"
    export function move(lspeed: number, rspeed: number) {
        moving=1
        leftspeed = MAX_SPEED * lspeed / 100;
        rightspeed = MAX_SPEED * rspeed / 100;
    }


    //% blockId=CarCtrl_straight block="移动|%index|速度 %speed |距离 %distence"
    //% weight=92
    //% group="移动控制"
    //% speed.min=0 speed.max=100
    export function go_line(index: CarMove, speed: number, distance: number): void {
        if (index == 1)
            base_controller.go_line(speed, distance)
        else
            base_controller.go_line(speed, -distance)

    }

    //% blockId=CarCtrl_turn block="移动|%index|速度 %speed |角度 %angle"
    //% weight=92
    //% group="移动控制"
    //% speed.min=0 speed.max=100
    export function turn(index: CarTurn, speed: number, angle: number): void {
        if (index == 1)
            base_controller.turn_in_place(speed, angle)
        else
            base_controller.turn_in_place(speed, -angle)
    }

    /** 移动到全局坐标系下的指定位姿(x,y,θ)，
     * @param x 目标点的x坐标
     * @param y 目标点的y坐标
     * @param theta 目标姿态角，逆时针为正，角度制°
     *            x ^
     *              |
     *              |
     *     y<———————|
    */
    //% blockId=Choco_move2 block="移动到 |x坐标 %x |y坐标 %y |方向 %angle |速度 %speed"
    //% weight=92
    //% inlineInputMode=inline
    //% speed.min=0 speed.max=100
    //% group="移动控制"
    export function CarMove2(x: number, y: number, theta: number, speed: number): void {
        base_controller.move2goal(speed, x, y, theta);
    }

    //% blockId=Choco_Servo block="舵机控制|编号 %num|角度 %value"
    //% weight=80
    //% advanced=true
    //% value.min=0 value.max=180
    //% num.fieldEditor="gridpicker" num.fieldOptions.columns=4
    export function Servo(num: enServo, value: number): void {



    }

    //% blockId=Choco_bus_Servo block="总线舵机控制|ID %ID|角度 %value|时间 %time ms"
    //% weight=79 color="#a5a5a5"
    //% time.defl=500 time.min=0
    //% value.min=0 value.max=180
    //% num.fieldEditor="gridpicker" num.fieldOptions.columns=4
    //% advanced=true
    export function bus_Servo(ID: number, value: number, time: number): void {

    }
    export enum Servo_mode {
        //% blockId=bus_Servo_mode_1 block="顺时针270°"
        CW270 = 1,
        //% blockId=bus_Servo_mode_2 block="逆时针270°"
        CCW,
        //% blockId=bus_Servo_mode_3 block="顺时针180°"
        CW180,
        //% blockId=bus_Servo_mode_4 block="逆时针180°"
        CCW180,
        //% blockId=bus_Servo_mode_5 block="顺时针圈数"
        CWN,
        //% blockId=bus_Servo_mode_6 block="逆时针圈数"
        CCWN,
        //% blockId=bus_Servo_mode_7 block="顺时针时间"
        CWT,
        //% blockId=bus_Servo_mode_8 block="逆时针时间"
        CCWT
    }

    //% blockId=Choco_bus_Servo_mode block="设置总线舵机模式|ID %ID| %mode"
    //% weight=78 color="#a5a5a5"
    //% advanced=true
    export function bus_Servo_mode(ID: number, mode: Servo_mode): void {

    }



    export enum IR_sensor {
        //% blockId=IR_Left2 block="左2"
        Left2 = 0,
        //% blockId=IR_Left1 block="左1"
        Left1,
        //% blockId=IR_Right1 block="右1"
        Right1,
        //% blockId=IR_Right2 block="右2"
        Right2

    }
    export enum IR_state {
        //% blockId=IR_black block="黑"
        black = 0,
        //% blockId=IR_white block="白"
        white = 1
    }

    //% blockId=Choco_IRsensor block="循线传感器 %n |检测到 %state" weight=50
    //% n.fieldEditor="gridpicker" n.fieldOptions.columns=4 color="#3d85c6" icon="\uf2f6"
    export function read_IRsensor(n: IR_sensor, state: IR_state): boolean {
        // let pin = (liner_buf[0] >> n) & 0x01;
        // if (pin == state) {
        //     return true;
        // }
        // else {
        //     return false;
        // }
        return true
    }

    //% blockId=Choco_rainbowlight block="启动流水灯 流动速度 %v |亮度 %brightness"
    //% weight=1 color="#6aa84f" icon="\uf0eb"
    //% v.defl=1 
    //% brightness.defl=1
    //% v.min=1 v.max=5 
    //% brightness.min=1 brightness.max=5
    export function rainbowlight(v: number, brightness: number) {
        let RGB: neopixel.Strip = null
        turn_off = false
        RGB = neopixel.create(DigitalPin.P5, 12, NeoPixelMode.RGB)
        RGB.setBrightness(10 * brightness)
        RGB.showRainbow(1, 360)
        control.inBackground(() => {
            while (!turn_off) {
                RGB.rotate(1)
                RGB.show()
                basic.pause(500 / v)
            }
            RGB.clear();
            RGB.show();
            basic.pause(100);
            pins.digitalReadPin(DigitalPin.P5)
            pins.setPull(DigitalPin.P5, PinPullMode.PullUp)
            while (1)
                basic.pause(60000);

        })
    }
    //% blockId=Choco_rainbowlight_off block="关闭流水灯"
    //% weight=1 color="#6aa84f" icon="\uf0eb"
    export function turnoff_rainbowlight() {
        turn_off = true
    }

    /**
     * 读取传感器数据.
     */
    //% blockId=i2c_readbuf block="更新传感器数据" weight=7 color="#3d85c6" weight=51
    export function Readbuf() {
        // let cmd = pins.createBuffer(5);
        // cmd[0] = 0xc1;
        // cmd[1] = 0;
        // cmd[2] = 0;
        // cmd[3] = 0
        // cmd[4] = 0
        // pins.i2cWriteBuffer(0x00, cmd);
        // liner_buf = pins.i2cReadBuffer(0x00, 5, false);
        // basic.pause(10);
        // cmd[0] = 0xc2;
        // pins.i2cWriteBuffer(0x00, cmd);
        // color_buf = pins.i2cReadBuffer(0x00, 3, false);
        // cmd[0] = 0xc3;
        // pins.i2cWriteBuffer(0x00, cmd);
        // odom_buf = pins.i2cReadBuffer(0x00, 6, false);
    }
    export enum RGB {
        //% blockId=red block="红"
        Red = 1,
        //% blockId=green block="绿" 
        Green = 2,
        //% blockId=yellow block="蓝"
        Yellow = 3
    }

    /**
     * 返回颜色传感器数据.0-红，1-黄，2-蓝，3-绿
     */
    //% blockId=get_color block="检测到%rgb|色？" weight=7
    export function get_color(rgb: RGB) {
        // switch(rgb){
        //     case 1:return color_buf[0];
        //     case 2:return color_buf[1];
        //     case 3:return color_buf[2];
        //     default:return 0;
        // }
    }

    export enum Gripper{
        //% blockId=Gripper_release block="松开"
        Release = 1,
        //% blockId=Gripper_catch block="夹紧"
        Catch,
        //% blockId=Gripper_catch block="抬起"
        UP,
        //% blockId=Gripper_catch block="放下"
        DOWN
    }
    /**
     * 机械爪控制
     */
    //% blockId=Gripper block="爪子 %pos" weight=10
    //% group="机械爪控制"
    export function gripper_control(pos:Gripper){
        let position = 0
        let time = 1000
        let cmd = pins.createBuffer(5);
        if(pos == Gripper.UP||pos == Gripper.DOWN)
        {
            cmd[0] = 0xc6;
            cmd[2] = 0;
            cmd[3] = 0;
            cmd[4] = 0;
            if(pos == Gripper.UP)
                cmd[1]=1
            else cmd[1]=2
            pins.i2cWriteBuffer(0x00, cmd);
            basic.pause(1000)
            cmd[1]=0
            pins.i2cWriteBuffer(0x00, cmd);
        }
        else
        {
            if(pos == Gripper.Release)
            position = 600
            else
            position = 1800
            cmd[0] = 0xc4;
            cmd[1] = position;
            cmd[2] = position>>8;
            cmd[3] = time;
            cmd[4] = time>>8
            pins.i2cWriteBuffer(0x00, cmd);
        }
    }
}