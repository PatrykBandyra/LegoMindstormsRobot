#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from ev3dev2.sound import Sound
from ev3dev2.motor import OUTPUT_A, OUTPUT_D, MoveTank, MediumMotor, SpeedRPM, SpeedPercent, Motor
from ev3dev2.sensor import INPUT_1, INPUT_2
from ev3dev2.sensor.lego import TouchSensor, ColorSensor, InfraredSensor
from time import sleep


class IRobot:

    LEFT_MOTOR = OUTPUT_A
    RIGHT_MOTOR = OUTPUT_D

    LEFT_LIGHT_SENSOR = INPUT_1
    RIGHT_LIGHT_SENSOR = INPUT_2

    CRANE_UP_TIME_IN_SECS = 2
    CRANE_DOWN_TIME_IN_SECS = 2

    MAX_SPEED_IN_RPM = 50 # CHANGE
    MIN_SPEED_IN_RPM = 0

    DISTANCE_TO_START_CRANE_IN_PCT = 1
    CRANE_SPEED_IN_PCT = 100

    def __init__(self):
        self.tank_drive = MoveTank(IRobot.LEFT_MOTOR, IRobot.RIGHT_MOTOR)
        self.crane = MediumMotor()
        self.sound = Sound()
        self.sound.set_volume(100)
        self.touch_sensor = TouchSensor()
        self.infrared_sensor = InfraredSensor()
        self.infrared_sensor.mode = InfraredSensor.MODE_IR_PROX
        self.color_sensor_left = ColorSensor(IRobot.LEFT_LIGHT_SENSOR)
        self.color_sensor_left.mode = ColorSensor.MODE_COL_COLOR
        self.color_sensor_right = ColorSensor(IRobot.RIGHT_LIGHT_SENSOR)
        self.color_sensor_right.mode = ColorSensor.MODE_COL_COLOR

        self.green_color = tuple()
        self.red_color = tuple()

        self.is_crane_up = False

        self.is_following_green = False
        self.is_following_red = False

        self.num_right_green_detected = 0
        self.num_right_green_not_detected = 0
        self.num_right_red_detected = 0
        self.num_right_red_not_detected = 0

        self.num_left_green_detected = 0
        self.num_left_green_not_detected = 0
        self.num_left_red_detected = 0
        self.num_left_red_not_detected = 0

        # 1 - left
        # 2 - right
        self.green_turn = 0
        self.was_green_turn = False

        # 1 - left
        # 2 - right
        self.red_turn = 0
        self.was_red_turn = False

        self.stop = False

        self.black_counter = 0

    def wait_for_touch_sensor_click(self):
        while not self.touch_sensor.value():
            sleep(0.01)

    def on_start(self):
        self.sound.beep()
        self.wait_for_touch_sensor_click()
        sleep(0.5)

    def on_finish(self):
        self.tank_drive.stop()
        self.crane_down()
        self.sound.beep()
        self.sound.speak('finished')


    def run(self):

        def clamp(value, max_value, min_value):
            return max(min(max_value, value), min_value)

        self.on_start()

        DRIVE_SPEED_IN_RPM = 20

        PROPORTIONAL_GAIN = 20
        INTEGRAL_GAIN = 0.000
        DERIVATIVE_GAIN = 2 # CHANGED

        integral = 0
        derivative = 0
        last_error = 0

        # Main loop can be finished by clicking a button or after a robot finishes all its tasks
        while not self.touch_sensor.value() and not self.stop:
            left_color = self.color_sensor_left.color
            right_color = self.color_sensor_right.color

            # Procedure: counting black color occurances to use it after leaving green branch on intersection with main road
            if left_color == right_color == 1:
                self.black_counter += 1
            else:
                self.black_counter = 0

            # Procedure: when robot is leaving green road and has to decide which direction to follow
            if not self.was_green_turn and self.is_following_green and self.black_counter > 5:
                if self.green_turn == 1:  # left
                    self.tank_drive.stop()
                    self.was_green_turn = True
                    self.tank_drive.on_for_degrees(SpeedRPM(0), SpeedRPM(20), 330)
                    continue
                elif self.green_turn == 2:  # right
                    self.tank_drive.stop()
                    self.was_green_turn = True
                    self.tank_drive.on_for_degrees(SpeedRPM(20), SpeedRPM(0), 330)
                    continue

            # Procedure: when robot is leaving red road and has to decide which direction to follow
            if not self.was_red_turn and self.is_following_red and self.black_counter > 5:
                if self.red_turn == 1:  # left
                    self.tank_drive.stop()
                    self.was_red_turn = True # CHANGE
                    self.tank_drive.on_for_degrees(SpeedRPM(0), SpeedRPM(20), 330)
                    continue
                elif self.red_turn == 2:  # right
                    self.tank_drive.stop()  # CHANGE
                    self.was_red_turn = True # CHANGE
                    self.tank_drive.on_for_degrees(SpeedRPM(20), SpeedRPM(0), 330)
                    continue

            # Counting how many times a given color has shown up in a row on a given side
            if left_color == 3:  # green
                self.num_left_green_detected += 1
            else:
                self.num_left_green_not_detected += 1
                if self.num_left_green_not_detected > 3:
                    self.num_left_green_detected = 0
                    self.num_left_green_not_detected = 0

            if left_color == 5:  # red
                self.num_left_red_detected += 1
            else:
                self.num_left_red_not_detected += 1
                if self.num_left_red_not_detected > 3:
                    self.num_left_red_detected = 0
                    self.num_left_red_not_detected = 0

            if right_color == 3:  # green
                self.num_right_green_detected += 1
            else:
                self.num_right_green_not_detected += 1
                if self.num_right_green_not_detected > 3:
                    self.num_right_green_detected = 0
                    self.num_right_green_not_detected = 0

            if right_color == 5:  # red
                self.num_right_red_detected += 1
            else:
                self.num_right_red_not_detected += 1
                if self.num_right_red_not_detected > 3:
                    self.num_right_red_detected = 0
                    self.num_right_red_not_detected = 0

            # Procedure: when green was detected first time
            if self.num_left_green_detected > 4 and not self.is_following_green:  # green CHANGE USED TO BE 5
                self.green_turn = 1
                self.is_following_green = True
                self.tank_drive.stop()
                self.tank_drive.on_for_degrees(SpeedRPM(0), SpeedRPM(20), 330)
                continue    

            elif self.num_right_green_detected > 4 and not self.is_following_green:  # green CHANGE USED TO BE 5
                self.green_turn = 2
                self.is_following_green = True
                self.tank_drive.stop()
                self.tank_drive.on_for_degrees(SpeedRPM(20), SpeedRPM(0), 330)
                continue  
            
            # Procedure: when green had been detected earlier and red was detected first time
            if self.is_following_green and self.num_left_red_detected > 4 and not self.is_following_red:
                self.red_turn = 1
                self.is_following_red = True
                self.tank_drive.stop()
                self.tank_drive.on_for_degrees(SpeedRPM(0), SpeedRPM(20), 330)
                continue    

            elif self.is_following_green and self.num_right_red_detected > 4 and not self.is_following_red:
                self.red_turn = 2
                self.is_following_red = True
                self.tank_drive.stop()
                self.tank_drive.on_for_degrees(SpeedRPM(20), SpeedRPM(0), 330)
                continue  
            

            # Procedure: when robot is following green road and detects its package in close proximity
            if self.is_following_green and not self.is_crane_up and self.infrared_sensor.proximity <= IRobot.DISTANCE_TO_START_CRANE_IN_PCT:
                sleep(0.5)
                self.tank_drive.stop()
                self.crane_up()
                self.tank_drive.on_for_degrees(SpeedRPM(-20), SpeedRPM(20), 330)

            # Procedure: when robot is following red road and detects big red area
            if self.is_following_red and self.num_left_red_detected > 3 and self.num_right_red_detected > 3:
                self.tank_drive.stop()
                self.crane_down()
                self.tank_drive.on_for_seconds(left_speed=SpeedRPM(-20), right_speed=SpeedRPM(-20), seconds=2)  # CHANGE 2 sec for longer track
                self.tank_drive.on_for_degrees(SpeedRPM(-20), SpeedRPM(20), 330)
                self.tank_drive.on_for_seconds(left_speed=SpeedRPM(20), right_speed=SpeedRPM(20), seconds=0.5)  # CHANGE 2 sec for longer track

            # PID controller for following the line
            error = right_color - left_color
            integral += error
            derivative = error - last_error

            turn_rate = PROPORTIONAL_GAIN * error + INTEGRAL_GAIN * integral + DERIVATIVE_GAIN * derivative

            speed_left = clamp(DRIVE_SPEED_IN_RPM - turn_rate, IRobot.MAX_SPEED_IN_RPM, IRobot.MIN_SPEED_IN_RPM)
            speed_right = clamp(DRIVE_SPEED_IN_RPM + turn_rate, IRobot.MAX_SPEED_IN_RPM, IRobot.MIN_SPEED_IN_RPM)

            self.tank_drive.on(left_speed=SpeedRPM(speed_left), right_speed=SpeedRPM(speed_right))

            last_error = error

            sleep(0.001)

        self.on_finish()


    def crane_up(self):
        if not self.is_crane_up:
            self.is_crane_up = True
            self.sound.beep()
            self.crane.on_for_seconds(speed=SpeedPercent(IRobot.CRANE_SPEED_IN_PCT), seconds=IRobot.CRANE_UP_TIME_IN_SECS)
            self.sound.beep()

    def crane_down(self):
        if self.is_crane_up:
            self.is_crane_up = False
            self.sound.beep()
            self.crane.on_for_seconds(speed=SpeedPercent(-IRobot.CRANE_SPEED_IN_PCT), seconds=IRobot.CRANE_DOWN_TIME_IN_SECS)
            self.sound.beep()


def main():
    IRobot().run()


if __name__ == '__main__':
    main()
