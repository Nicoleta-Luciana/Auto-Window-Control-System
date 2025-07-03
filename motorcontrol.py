import smbus
import time
import RPi.GPIO as GPIO

# Configurare părți comune
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# ----------------------------------
# Configurare motor DC
# ----------------------------------
RIGHT_PIN = 13
LEFT_PIN = 15
PROXIMITY_PIN = 18

pwm_pin = 32
motor_dir_pin = 11

GPIO.setup(motor_dir_pin, GPIO.OUT)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(RIGHT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(LEFT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PROXIMITY_PIN, GPIO.IN)

pwm = GPIO.PWM(pwm_pin, 1000)  # Frecvență 1000 Hz
pwm.start(0)

def rotate_right(speed):
    GPIO.output(motor_dir_pin, GPIO.HIGH)
    pwm.ChangeDutyCycle(speed)

def rotate_left(speed):
    GPIO.output(motor_dir_pin, GPIO.LOW)
    pwm.ChangeDutyCycle(speed)

def stop_motor():
    pwm.ChangeDutyCycle(0)

# ----------------------------------
# Configurare LCD
# ----------------------------------
I2C_ADDR = 0x27  # Adresa I2C a LCD-ului
LCD_WIDTH = 16

LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0

ENABLE = 0b00000100
BACKLIGHT = 0b00001000

SWITCH = 11
GPIO.setup(SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bus = smbus.SMBus(1)

def lcd_write(bits, mode):
    high_bits = mode | (bits & 0xF0) | BACKLIGHT
    low_bits = mode | ((bits << 4) & 0xF0) | BACKLIGHT

    bus.write_byte(I2C_ADDR, high_bits)
    lcd_toggle_enable(high_bits)
    bus.write_byte(I2C_ADDR, low_bits)
    lcd_toggle_enable(low_bits)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_init():
    lcd_write(0x33, LCD_CMD)
    lcd_write(0x32, LCD_CMD)
    lcd_write(0x06, LCD_CMD)
    lcd_write(0x0C, LCD_CMD)
    lcd_write(0x28, LCD_CMD)
    lcd_write(0x01, LCD_CMD)
    time.sleep(0.0005)

def lcd_message(message):
    message = message.ljust(LCD_WIDTH, " ")
    for char in message:
        lcd_write(ord(char), LCD_CHR)

def lcd_clear():
    lcd_write(0x01, LCD_CMD)

# ----------------------------------
# Funcții independente pentru motor și LCD
# ----------------------------------

def motor_control():
    if GPIO.input(RIGHT_PIN) == GPIO.LOW:
        print("Rotire la dreapta")
        rotate_right(50)

    elif GPIO.input(LEFT_PIN) == GPIO.LOW:
        if GPIO.input(PROXIMITY_PIN) == GPIO.HIGH:
            print("Senzor de proximitate activat! Motor oprit.")
            stop_motor()
            time.sleep(1)
            print("Inversare direcție: Rotire la dreapta")
            rotate_right(50)
        else:
            print("Rotire la stânga")
            rotate_left(50)
    else:
        stop_motor()


def lcd_control():
    if GPIO.input(SWITCH):
        lcd_clear()
        lcd_write(LCD_LINE_1, LCD_CMD)
        lcd_message("Dreapta")
    else:
        lcd_clear()
        lcd_write(LCD_LINE_1, LCD_CMD)
        lcd_message("Stanga")

# ----------------------------------
# Program principal
# ----------------------------------

def main():
    lcd_init()  # Inițializează LCD-ul

    try:
        while True:
            motor_control()  # Controlează motorul independent
            lcd_control()    # Controlează LCD-ul independent
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram oprit de utilizator.")
    finally:
        lcd_clear()
        pwm.stop()
        GPIO.cleanup()

if _name_ == "_main_":
    main()