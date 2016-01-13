#!/usr/bin/env python

import remi.gui as gui
from remi import start, App
import rospy
from geometry_msgs.msg import Twist
import threading
import time

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0
PUB_PERIOD = 1.0/10.0  # 10hz
PING_PERIOD = 1.0

class HeartbeatWidget(gui.Tag):
    def __init__(self, beat_rate_millisec=1000, timeout_millisec=1500 ):
        super(HeartbeatWidget, self).__init__()

        self.timeout_millisec = timeout_millisec
        self.beat_rate_millisec = beat_rate_millisec

        self.eventManager = gui.EventManager()
        self.type = 'script'
        self.attributes['type'] = 'text/javascript'
        self.EVENT_ONBEAT = 'onbeat'
        self.EVENT_ONTIMEOUT = 'ontimeout'
        self.EVENT_ONRECONNECT = 'onreconnect'
        self.append( 'javascript_code' , """
            function sendHeartBeat%(id)s(){
                sendCallback('%(id)s','%(event)s');
            };
            setInterval( sendHeartBeat%(id)s, %(beat_rate)s );
            """ % {'id':id(self), 'beat_rate':beat_rate_millisec, 'event':self.EVENT_ONBEAT })

        self.last_beat_millisec = -1
        self.disconnected = True

    def onbeat(self):
        #print('beat')
        self.last_beat_millisec = time.time()*1000
        threading.Timer(self.timeout_millisec/1000.0, self.check_timeout).start()
        if self.disconnected:
            self.disconnected = False
            self.eventManager.propagate(self.EVENT_ONRECONNECT, [])
            print('We were disconnected, reconnecting!')

    def check_timeout(self):
        #print('Checking timeout...')
        tnow = time.time()*1000
        if (tnow - self.last_beat_millisec) > self.beat_rate_millisec:
            self.eventManager.propagate(self.EVENT_ONTIMEOUT, [])
            self.disconnected = True
            print('We are too late! disconnected!')

    def set_on_timeout_listener(self, listener, funcname):
        self.eventManager.register_listener(self.EVENT_ONTIMEOUT, listener, funcname)

    def set_on_reconnect_listener(self, listener, funcname):
        self.eventManager.register_listener(self.EVENT_ONRECONNECT, listener, funcname)

class JoystickWidget(gui.Svg):
    def __init__(self, width, height):
        super(JoystickWidget, self).__init__(width, height)
        self.width = width
        self.height = height
        # self.maxDragLength = min(width, height)/3.0*2.0
        self.maxDragLength = min(width, height)/3.0
        self.startPointIndicator = gui.SvgCircle(0, 0, 20)
        self.startPointIndicator.set_fill('rgb(255,200,50)')
        self.startPointIndicator.set_stroke(1, 'white')
        self.endPointIndicator = gui.SvgCircle(0, 0, 10)
        self.endPointIndicator.set_fill('rgb(200,255,50)')
        self.endPointIndicator.set_stroke(1, 'white')
        self.pathLine = gui.SvgLine(0, 0, 0, 0)
        self.startX = 0
        self.startY = 0
        self.append('path_line', self.pathLine)
        self.append('start_point', self.startPointIndicator)
        self.append('end_point', self.endPointIndicator)
        self.drag_state = False
        self.set_on_mouseup_listener(self, 'mouseup')
        self.set_on_mousedown_listener(self, 'mousedown')
        self.set_on_mousemove_listener(self, 'mousemove')
        self.set_on_touchend_listener(self, 'mouseup')
        self.set_on_touchstart_listener(self, 'mousedown')
        self.set_on_touchmove_listener(self, 'mousemove')

        self.EVENT_ONMOVE = "ONJOYSTICKMOVE"
        self.reset_joystick(0, 0)

    def mousemove(self, x, y):
        if self.drag_state:
            self.endPointIndicator.set_position(x, y)
            self.pathLine.set_p2(x, y)
            moveX = min(abs(x-self.startX),self.maxDragLength)/self.maxDragLength * (-1 if x < self.startX else 1)
            moveY = min(abs(y-self.startY),self.maxDragLength)/self.maxDragLength * (1 if y < self.startY else -1)
            params = [moveX,moveY]
            self.eventManager.propagate(self.EVENT_ONMOVE, params)
        
    def mousedown(self, x, y):
        self.drag_state = True
        #self.startPointIndicator.set_position(x, y)
        self.startPointIndicator.set_position(self.width/2, self.height/2)
        #self.pathLine.set_p1(x, y)
        self.pathLine.set_p1(self.width/2, self.height/2)
        # self.startX = x
        # self.startY = y
        self.startX = self.width/2
        self.startY = self.height/2

    def mouseup(self, x, y):
        self.reset_joystick(x, y)

    def reset_joystick(self, x, y):
        self.drag_state = False
        self.startPointIndicator.set_position(self.width/2, self.height/2)
        self.endPointIndicator.set_position(self.width/2, self.height/2)
        self.pathLine.set_coords(self.width/2, self.height/2, self.width/2, self.height/2)
        params = [0,0]
        self.eventManager.propagate(self.EVENT_ONMOVE, params)

    def set_onmove_listener(self, listener, funcname):
        self.eventManager.register_listener(self.EVENT_ONMOVE, listener, funcname)


class MyApp(App):

    def __init__(self, *args, **kwargs):
        super(MyApp, self).__init__(*args, **kwargs)

    def main(self, name='world'):
        self.twist_pub = rospy.Publisher('/key_vel', Twist, queue_size=1)
        rospy.Timer(rospy.Duration(PUB_PERIOD), self.pub_twist)
        self.wid = gui.Widget(620, 650, False, 10)

        self.infoLabel = gui.Label(300, 30, "coords")
        self.last_x = 0.0
        self.last_y = 0.0
        self.joystick = JoystickWidget(600, 600)
        self.joystick.style['background-color'] = 'rgb(255,255,255)'
        self.joystick.set_onmove_listener(self, "joystick_moved")

        self.wid.append('infolabel', self.infoLabel)
        self.wid.append('joystick', self.joystick)
        self.wid.set_on_mousemove_listener(self.joystick, 'reset_joystick')

        self.client_connected = True
        self.heartbeat = HeartbeatWidget()
        self.heartbeat.set_on_timeout_listener(self, "disconnection")
        self.heartbeat.set_on_reconnect_listener(self, "reconnection")
        self.wid.append('heartbeat', self.heartbeat)

        # returning the root widget
        return self.wid

    def disconnection(self):
        rospy.logerr("Disconnection of the client!")
        self.client_connected = False

    def reconnection(self):
        rospy.logwarn("Reconnection of the client!")
        self.client_connected = True

    def joystick_moved(self, x, y):  # 0 <= x <= 1     0 <= y <= 1
        self.infoLabel.set_text("move x:%s  y:%s" % (x, y))
        self.last_x = x
        self.last_y = y

    def pub_twist(self, timerevent):
        if self.joystick.drag_state and self.client_connected:
            t = Twist()
            if self.last_y >= 0:
                t.linear.x = self.last_y * MAX_LINEAR_SPEED
            else:
                t.linear.x = self.last_y * MAX_ANGULAR_SPEED / 2.0
            t.angular.z = -self.last_x * MAX_ANGULAR_SPEED
            self.twist_pub.publish(t)


if __name__ == "__main__":
    rospy.init_node('remi_joy')
    start(MyApp,
          address='0.0.0.0',
          multiple_instance=True,
          port=8092,
          websocket_timeout_timer_ms=1000,
          pending_messages_queue_length=1000)
