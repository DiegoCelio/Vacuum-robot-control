# ~/ros2_ws/src/vacuum_control/vacuum_control/gpio_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

GPIO_PIN_NUMBER_V = 27  # Replace with the GPIO pin you want to control with key 'v'
GPIO_PIN_NUMBER_R = 17  # Replace with the GPIO pin you want to control with key 'r'
GPIO_PIN_NUMBER_L = 22  # Replace with the GPIO pin you want to control with key 'l'

class GpioSubscriber(Node):
    def __init__(self):
        super().__init__('gpio_subscriber')
        self.subscription = self.create_subscription(
            String,
            'key_pressed',
            self.callback,
            10
        )
        self.setup_gpio()

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(GPIO_PIN_NUMBER_V, GPIO.OUT)
            GPIO.setup(GPIO_PIN_NUMBER_R, GPIO.OUT)
            GPIO.setup(GPIO_PIN_NUMBER_L, GPIO.OUT)
            GPIO.output(GPIO_PIN_NUMBER_V, GPIO.LOW)  # Initialize the pin to be OFF
            GPIO.output(GPIO_PIN_NUMBER_R, GPIO.LOW)  # Initialize the pin to be OFF
            GPIO.output(GPIO_PIN_NUMBER_L, GPIO.LOW)  # Initialize the pin to be OFF
        except Exception as e:
            self.get_logger().error(f'Error setting up GPIO: {e}')
            rclpy.shutdown()

    def callback(self, msg):
        try:
            if msg.data == 'v':
                if GPIO.input(GPIO_PIN_NUMBER_V):
                    self.get_logger().info('Deactivating GPIO pin 27')
                    GPIO.output(GPIO_PIN_NUMBER_V, GPIO.LOW)  # Deactivate GPIO pin if already active
                else:
                    self.get_logger().info('Activating GPIO pin 27')
                    GPIO.output(GPIO_PIN_NUMBER_V, GPIO.HIGH)  # Activate GPIO pin if it is off
            elif msg.data == 'r':
                if GPIO.input(GPIO_PIN_NUMBER_R):
                    self.get_logger().info('Deactivating GPIO pin 17')
                    GPIO.output(GPIO_PIN_NUMBER_R, GPIO.LOW)  # Deactivate GPIO pin if already active
                else:
                    self.get_logger().info('Activating GPIO pin 17')
                    GPIO.output(GPIO_PIN_NUMBER_R, GPIO.HIGH)  # Activate GPIO pin if it is off
            elif msg.data == 'l':
                if GPIO.input(GPIO_PIN_NUMBER_L):
                    self.get_logger().info('Deactivating GPIO pin 22')
                    GPIO.output(GPIO_PIN_NUMBER_L, GPIO.LOW)  # Deactivate GPIO pin if already active
                else:
                    self.get_logger().info('Activating GPIO pin 22')
                    GPIO.output(GPIO_PIN_NUMBER_L, GPIO.HIGH)  # Activate GPIO pin if it is off
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')

    def cleanup_gpio(self):
        GPIO.cleanup()

    def on_shutdown(self):
        self.cleanup_gpio()

def main(args=None):
    rclpy.init(args=args)
    node = GpioSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

