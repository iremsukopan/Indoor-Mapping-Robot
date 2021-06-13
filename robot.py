
from gpiozero import Robot
import time
import RPi.GPIO as GPIO
import turtle
import smbus
import math
from PIL import Image
from http.server import BaseHTTPRequestHandler, HTTPServer
import base64

# pins are in BCM
GPIO.setmode(GPIO.BCM)

# DISTANCE SENSOR PINS
f_trig = 5
f_echo = 21
r_trig = 23
r_echo = 24

check = 0
curve = 0
bump_in_timer = 0
bump_out_timer = 0
front_wall_timer = 0
# ACCELEROMETER
address = 0x53
bus = smbus.SMBus(1)
bus.write_byte_data(address, 0x2C, 0x0A)
bus.write_byte_data(address, 0x2D, 0x08)
bus.write_byte_data(address, 0x31, 0x08)

state = 0

# FIND A WALL
f_treshold = 35
r_treshold = 70  # in cm

front_wall_detected = False
right_wall_detected = False

# MOTOR DRIVER PINS
in1 = 16
in2 = 25
in3 = 22
in4 = 27

robot = Robot(left=(in4, in3), right=(in2, in1))

turn_angle = 90  # for turtle
scale = 1  # for turtle

counter = 0
temp1 = 0

orientation_list = []
turn_list = []
wall_length = []
orientation = 0
coordinate = [0, 0]

left = 1
right = -1
distance = 0
v_init = 22
end_time = time.time()
change = 0

previous_length = 0

away_time = 0
closer_time = 0


# GET DISTANCE DATA
def get_dist(myTrig, myEcho):  # getting data from distance sensor

    TRIG = myTrig
    ECHO = myEcho

    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)

    #     if TRIG == f_trig:
    #         print("Front distance:", distance, "cm")
    #    if TRIG == r_trig:
    #         print("Right distance:", distance, "cm")

    return distance


# GET ACCELERATION
def get_accel(address):
    data0 = bus.read_byte_data(address, 0x34)
    data1 = bus.read_byte_data(address, 0x35)

    # Convert the data to 10-bits
    yAccl = ((data1 & 0x03) * 256) + data0
    if yAccl > 511:
        yAccl -= 1024
    #    print("Acceleration in Y-Axis : ", -yAccl)
    return -yAccl


def completion(counter, wall_length, turn_list, turn_angle):  # map creation algorithm
    robot.stop()
    global turtle
    global t
    t = turtle.Turtle()
    #    print(len(wall_length))
    #    print(len(turn_list))
    for i in range(len(wall_length)):  # creating map
        print("calculating wall ", i + 1, ", length  ", int(wall_length[i]), " cm")
        t.forward(wall_length[i] * scale)
        if turn_list[i] == left:
            t.left(turn_angle)
        elif turn_list[i] == right:
            t.right(turn_angle)
    print("Done.")
    sc = turtle.getscreen()
    sc.getcanvas().postscript(file="map.eps")
    #    turtle.done()
    fileName = 'map.eps'
    screen = turtle.Screen()
    screen.setup(1000, 1000)
    turtle = turtle.Turtle(visible=False)
    screen.tracer(False)
    screen.tracer(True)
    canvas = screen.getcanvas()
    canvas.postscript(file=fileName)
    canvas.postscript(file=fileName, width=1000, height=1000)

    # converting .eps to .jpg
    img = Image.open(fileName)
    img.save("map.jpg", "JPEG")
    print("Saved")
    counter += 1
    return counter


def findWalls(r_trig, r_echo, f_trig, f_echo):  # how the robot initialy finds walls
    right_wall_detected = True

    if not get_dist(r_trig, r_echo) <= r_treshold:
        right_wall_detected = False
    #    if get_dist(f_trig,f_echo) <= 30:
    #        robot.left(0.7)
    #        time.sleep(0.2)
    while get_dist(f_trig, f_echo) <= 400 and 30 <= get_dist(f_trig, f_echo) and not right_wall_detected:
        #        print("Getting closer to front wall")
        robot.forward(0.7)
        time.sleep(0.2)
    while not right_wall_detected:
        #        print("looking for wall...")
        right_wall_detected = where_is_my_wall(r_trig, r_echo, f_trig, f_echo)
    return right_wall_detected


def where_is_my_wall(r_trig, r_echo, f_trig, f_echo):  # wall finding 360 loop
    robot.left(0.4)
    #    print("Searching for wall...")
    time.sleep(0.1)
    robot.stop()
    right_wall_detected = False
    if get_dist(r_trig, r_echo) <= r_treshold:
        right_wall_detected = True
        robot.left(0.4)
        time.sleep(0.1)
        robot.stop()
    return right_wall_detected


def move_closer():  # keep treshold distance
    robot.right(0.4)
    time.sleep(0.1)
    #    robot.left(0.3)
    #    time.sleep(0.1)
    #    robot.stop
    return


def move_away():  # keep treshold distance
    robot.right(0.4)
    time.sleep(0.1)
    #    robot.right(0.3)
    #    time.sleep(0.1)
    #    robot.stop
    return


while counter == 0:  # if completion hasnt been reached
    right_wall_detected = findWalls(r_trig, r_echo, f_trig, f_echo)  # find right wall
    previousRight = get_dist(r_trig, r_echo)  # for keeping the wall at a fixed distance and bump in/out handling
    next_time = time.time()  # for completion
    while right_wall_detected and counter == 0:
        time.sleep(0.1)
        RightDistance = get_dist(r_trig, r_echo)
        if RightDistance > 100:  # finding right wall by turning right
            right_wall_detected = False
            print("Right wall lost...")
            temp = 0
        while not right_wall_detected:
            robot.right(0.5)
            print("searching for LOST wall")
            time.sleep(0.1)
            robot.stop()
            if temp == 3:
                robot.forward(0.7)
                time.sleep(0.5)
                robot.stop()
            temp += 1
            RightDistance = get_dist(r_trig, r_echo)
            if RightDistance <= r_treshold:
                right_wall_detected = True
        if RightDistance <= 10 and 3 <= end_time - away_time:  # robot too close to the wall
            move_away()
            away_time = time.time()
        if 40 <= RightDistance and 3 <= end_time - closer_time:  # robot to far from the wall
            move_closer()
            closer_time = time.time()
        robot.forward(0.7, curve_left=curve)
        if state == 0:  # only once a wall
            print("Cruising...")
            state = 1
        #        time.sleep(0.15)
        start_time = end_time
        end_time = time.time()
        elapsed_time = end_time - (start_time)
        accel = get_accel(address) / 100  # conversion
        distance += elapsed_time * (accel * elapsed_time / 2 + v_init)  # calculate wall length
        v_init += elapsed_time * accel  # update velocity

        FrontDistance = get_dist(f_trig, f_echo)
        if FrontDistance <= f_treshold:  # front wall detected
            front_wall_detected = True
            robot.stop()
            print("!!FRONT WALL DETECTED!!")
            next_wall_distance = get_dist(r_trig, r_echo)
            robot.stop()
            right_wall_detected = False
            robot.left(0.9)
            time.sleep(0.2)
            robot.stop()
            while not right_wall_detected:
                right_wall_detected = where_is_my_wall(r_trig, r_echo, f_trig, f_echo)
            #            robot.right(0.2)
            #            time.sleep(0.05)
            wall_length.append(distance + FrontDistance)
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(left)
            orientation += left
            distance = next_wall_distance
            front_wall_timer = 0
            v_init = 22
            state = 0

        if 20 <= abs(RightDistance - previousRight) and abs(RightDistance - previousRight) <= 40:  # is it a bump?
            change = RightDistance - previousRight
        previousRight = RightDistance
        # continue cruising
        # If there is a small bump on the route
        if r_treshold > change > 10 and 3 < time.time() - bump_out_timer and 3 < time.time() - front_wall_timer:  # bump handling
            time.sleep(0.5)
            check = get_dist(r_trig, r_echo)
            if abs(check - RightDistance) < 5:
                new_wall = change
                orientation += right
                orientation = orientation % 4
                orientation_list.append(orientation)
                turn_list.append(right)
                wall_length.append(new_wall * scale)
                orientation += left
                turn_list.append(left)
                distance = 0
                bump_out_timer = time.time()

        elif -30 < change < -10 and 3 < time.time() - bump_in_timer and 3 < time.time() - front_wall_timer:  # bump handling
            time.sleep(0.5)
            check = get_dist(r_trig, r_echo)
            if abs(check - RightDistance) < 5:
                new_wall = -change
                orientation += left
                orientation = orientation % 4
                orientation_list.append(orientation)
                turn_list.append(left)
                wall_length.append(new_wall * scale)
                orientation += right
                turn_list.append(right)
                distance = 0
                bump_in_timer = time.time()
        if 30 < time.time() - next_time:  # after robot cruises for 30 secs
            counter = completion(counter, wall_length, turn_list, turn_angle)  # draw map
        continue

    # READ THE JPG FILE AND ENCODE IT
fileName = 'map.jpg'

# encode
with open(fileName, "rb") as image_file:
    encoded_string = base64.b64encode(image_file.read())


class RequestHandler_httpd(BaseHTTPRequestHandler):
    def do_GET(self):
        message_to_send = encoded_string
        self.send_response(200)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', len(message_to_send))
        self.end_headers()
        self.wfile.write(message_to_send)
        return


# create server connection and send encoded string
server_address_httpd = ('192.168.68.118', 8080)
httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
print('Starting server: ')
httpd.serve_forever()
