###############################################
# Imports                                     #
###############################################
import time
import RPi.GPIO as GPIO

class ConnectorActuation:
    def __init__(self):
        #self.connectorOpen = True
        GPIO.setmode(GPIO.BOARD)
        self.gpio_pin = 37

        GPIO.setup(self.gpio_pin, GPIO.OUT)
        GPIO.output(self.gpio_pin, 0)
        
    def close_connector(self):
        GPIO.output(self.gpio_pin, 0)
        print("Real-world connector closed")

    def open_connector(self):
        GPIO.output(self.gpio_pin, GPIO.HIGH)
        print("open")
        time.sleep(3)
        GPIO.output(self.gpio_pin, GPIO.LOW)
        print("closed")
        print("Real-world connector opened")
        

if __name__ == '__main__':
    ca = ConnectorActuation()
    ca.open_connector()
    print("finished")
    GPIO.cleanup()