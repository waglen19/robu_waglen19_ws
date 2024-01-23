# Parameter sind Variablen, die veränderbar sind, während das Programm läuft
 
# Parameter im Terminal
# ros2 param list .... gibt alle parameter aus
# ros2 param get <Node> <Parameter> .... git den Wert des Parameters aus
# ros2 param set <Node> <Parameter> <Wert> .... setzt den Wert des Parameters
 
 
# Datein importieren fürs senden und empfangen von Ros Datein ................................................
import rclpy                                                        #Ros schnittstelle für Python
from rclpy.node import Node
#.............................................................................................................
 
# Klasse erstellen ...........................................................................................
class MinimalParameter(Node):
    #++++ Konstruktor ++++
    def __init__(self):
        # super: auf Elternklasse zugreifen
        # Konstruktor der Elternklasse(NODE) aufrufen und Namen der Node vergeben
        super().__init__('MinimalParameter')
 
        # Parameter Beschreibung
        from rcl_interfaces.msgs import ParameterDescriptor
        my_parameter_description = ParameterDescriptor(description='This Parameter is mine!')
 
        # Parameter erstellen:      name        wert         beschreibung
        self.declare_parameters(namespace='',
                                parameters=[
            ('forward_speed_wf_slow', 0.05),
            ('forward_speed_wf_fast', 0.01),
            ('turning_speed_wf_slow', 0.01),
            ('turning_speed_wf_fast', 1.0),
            ('dist_tresh_wf', 0.3),
            ('dist_hysteresis_wf', 0.02),
            ])

 
        # Parameter lesen
        # gibt das Objekt des Parameters zurück
        #my_param = self.get_parameter('my_parameter')
        # gibt den Wert des Parameters zurück
        #my_param = self.get_parameter('my_parameter').get_parameter_value().string_value
 
        #print("my_parameter: ", my_param)
 
        self.timer = self.create_timer(1.0, self.timer_callback)
 
    def timer_callback(self):
        forward_speed_wf_slow = self.get_parameter('forward_speed_wf_slow').value
        forward_speed_wf_fast = self.get_parameter('forward_speed_wf_fast').value
        turning_speed_wf_fast = self.get_parameter('turning_speed_wf_fast').value
        turning_speed_wf_slow = self.get_parameter('turning_speed_wf_slow').value
        dist_tresh_wf = self.get_parameter('dist_tresh_wf').value
        dist_hysteresis_wf = self.get_parameter('dist_tresh_wf').value
 
        #Ausgabe der Werte am Bildschirm -> kann auch mit print gemacht werden, ROS hat aber viele verschiedene Möglichkeiten
        #Syntax für die Methoden print und self.get_logger().info
        #('Formatstring' % (argmuent1, argument2, argument3))
        self.get_logger().info('forward_speed_wf_slow: %5.2f forward_speed_wf_fast: %5.2f' 
                               % (forward_speed_wf_slow, forward_speed_wf_fast))
        self.get_logger().info('turning_speed_wf_fast: %5.2f turning_speed_wf_slow: %5.2f'
                               % (turning_speed_wf_fast, turning_speed_wf_slow))
        self.get_logger().info('dist_tresh_wf:         %5.2f dist_hysteresis_wf:    %5.2f'
                               % (dist_tresh_wf, dist_hysteresis_wf))
 
        #print("my_parameter: ", my_param)
        # self.get_logger().info('Hello %s!', my_param)
#.............................................................................................................
 
# Main .......................................................................................................
def main():
    rclpy.init()
 
    node = MinimalParameter()
 
    rclpy.spin(node)
#.............................................................................................................
 
if __name__ == '__main__':
    main()

    