import RPi.GPIO as GPIO

def clean():
    GPIO.cleanup()

if __name__ == '__main__':
    clean()