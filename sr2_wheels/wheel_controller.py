import rclpy
from rclpy.node import Node
from sr2_wheels.PyRoboteq.roboteq_handler import RoboteqHandler
from sr2_wheels.PyRoboteq import roboteq_commands as cmds
import math

from geometry_msgs.msg import Twist


class WheelController(Node):

    SPEED = 1000

    class Wheel:
        FRONT_RIGHT = '/dev/ttyACM0'
        FRONT_LEFT = '/dev/ttyACM1'
        BACK_LEFT = '/dev/ttyACM2'
        BACK_RIGHT = '/dev/ttyACM3'

        def __init__(self, wheel, debug=True, exit_on_interupt=False) -> None:
            self.handler = RoboteqHandler(debug_mode=debug, exit_on_interrupt=exit_on_interupt)
            self.handler.connect(wheel)

        def send_cmd(self, cmd_val):
            self.handler.send_command(cmds.SET_SPEED, 0, cmd_val)

    def __init__(self):
        super().__init__('wheel_controller')

        self._fr_wheel = self.Wheel(self.Wheel.FRONT_RIGHT)
        self._fl_wheel = self.Wheel(self.Wheel.FRONT_LEFT)
        self._br_wheel = self.Wheel(self.Wheel.BACK_RIGHT)
        self._bl_wheel = self.Wheel(self.Wheel.BACK_LEFT)

        self._subscriber = self.create_subscription(Twist, "sr2/wheels/cmd_vel", self.callback_function, 10)

    
    def callback_function(self, msg):
        self.get_logger().info("Message Recieved")
        x = msg.linear.x
        y = msg.linear.y
        r = msg.angular.z

        vfr, vfl, vbr, vbl = self._calculate_angles(x, y, r)

        self._fr_wheel.send_cmd(self.SPEED*vfr)
        self._fl_wheel.send_cmd(-self.SPEED*vfl)
        self._br_wheel.send_cmd(self.SPEED*vbr)
        self._bl_wheel.send_cmd(-self.SPEED*vbl)

    def _calculate_angles(self, x, y, r):
        def v_fl_calc(vd,td,vt):
            return vd*math.sin(td+(math.pi/4))+vt

        def v_fr_calc(vd,td,vt):
            return vd*math.sin(td-(math.pi/4))-vt
        
        def v_bl_calc(vd,td,vt):
            return vd*math.sin(td-(math.pi/4))+vt

        def v_br_calc(vd,td,vt):
            return vd*math.sin(td+(math.pi/4))-vt

        def vd_calc(x, y):
            if x == 0:
                return abs(y)
            elif y == 0:
                return abs(x)
            else:
                return math.sqrt(x*x +y*y)/(math.sqrt(2)/2)*0.5
        

        vd = vd_calc(x, y)

        if y == 0 and x != 0:
            if x > 0:
                td=0
            else:
                td=math.pi  
        elif x == 0 and y != 0:
            if y > 0:
                td = math.pi/2
            else:
                td = 3*math.pi/2
        elif x > 0 and y > 0:
            td = math.atan(y/x)
        elif x < 0 and y > 0:
            td = math.atan(y/x) + math.pi
        elif x < 0 and y < 0:
            td = math.atan(y/x) + math.pi
        elif x > 0 and y < 0:
            td = math.atan(y/x) + 2*math.pi
        else:
            td = 0

        v_fr = v_fr_calc(vd, td, r)
        v_fl = v_fl_calc(vd, td, r)
        v_br = v_br_calc(vd, td, r)
        v_bl = v_bl_calc(vd, td, r)

        return v_fr, v_fl, v_br, v_bl


def main(args=None):
    rclpy.init(args=args)

    node = WheelController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()