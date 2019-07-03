import RPi.GPIO as GPIO
from threading import Thread
from flask import Flask, request
import smbus, pygame, time

app = Flask(__name__)


I2C_ADDR = 0x3f             # dia chi module lcd
LCD_WIDTH = 16              # so ky tu tren mot dong

LCD_CHR = 1                 # gui ky tu
LCD_CMD = 0                 # gui lenh

LCD_LINE_1 = 0x80           # dia chi ram dong 1
LCD_LINE_2 = 0xC0           # dia chi ram dong 2

LCD_BACKLIGHT = 0x08        # on
#LCD_BACKLIGHT = 0x00       # off

ENABLE = 0b00000100         # bat bit
E_PULSE = 0.0005
E_DELAY = 0.0005

bus = smbus.SMBus(1)

def lcd_init():
    # Initialise display
    lcd_byte(0x33,LCD_CMD) 
    lcd_byte(0x32,LCD_CMD)  
    lcd_byte(0x06,LCD_CMD)  
    lcd_byte(0x0C,LCD_CMD)  
    lcd_byte(0x28,LCD_CMD)  
    lcd_byte(0x01,LCD_CMD)  
    time.sleep(E_DELAY)

# gui 1 byte xuong LCD
def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT
    #che do 4 bits: gui byte cao truoc byte thap sau
    # byte cao
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)
    # byte thap
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

# dua chan E len cao roi thap de truyen du lieu di
def lcd_toggle_enable(bits):
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
    time.sleep(E_DELAY)

# gui chuoi ki tu xuong LCD
def lcd_string(message,line):
    message = message.ljust(LCD_WIDTH," ")
    lcd_byte(line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(ord(message[i]),LCD_CHR)

GPIO.setmode(GPIO.BCM)
LED_PIN = [17]

def led_pin_init():
    for i in range(0,len(LED_PIN)):
        GPIO.setup(LED_PIN[i], GPIO.OUT)
        GPIO.output(LED_PIN[i], 0)

global flag_auto_run
flag_auto_run = True

def auto_run():
    while True:
        if flag_auto_run:
            for i in range(0,len(LED_PIN)):
                GPIO.output(LED_PIN[i], 1)
                print('Led', i , 'On')
                time.sleep(1)
                GPIO.output(LED_PIN[i], 0)
                print('Led', i , 'Off')
                time.sleep(1)
        else:
            continue

@app.route("/")
@app.route("/mode")
def change_mode():
    global flag_auto_run
    flag_auto_run = eval(request.args.get('auto'))
    if flag_auto_run:
        pygame.mixer.music.load("bugs_11.wav")
        pygame.mixer.music.play()
        lcd_string("On Auto Mode", LCD_LINE_1)
        return ('On Auto Mode')
    else:
        flag_auto_run = False
        pygame.mixer.music.load("bugs_07.wav")
        pygame.mixer.music.play()
        lcd_string("On Control Mode", LCD_LINE_1)
        return ('On Control Mode')
    
@app.route("/control/<led_id>")
def control_mode(led_id):
    global flag_auto_run
    if flag_auto_run:
        lcd_string("On Auto Mode", LCD_LINE_1)
        return ('IS RUNNING AUTO MODE, PLEASE SETUP THE CONTROL MODE')
    else:
        status_request = eval(request.args.get('status'))
        port = LED_PIN[int(led_id)]
        if status_request:
            GPIO.output(port, 1)
            print('TURN ON', led_id)
            lcd_string("Turn On " + led_id, LCD_LINE_1)
            return ('TURN ON  ' + led_id)
        else:
            GPIO.output(port, 0)
            print('TURN OFF', led_id)
            lcd_string("Turn On " + led_id, LCD_LINE_1)
            return('TURN OFF  ' + led_id) 


if __name__ == "__main__":
    led_pin_init()
    pygame.init()
    lcd_string("On Auto Mode", LCD_LINE_1)
    pygame.mixer.music.load("bugs_11.wav")
    pygame.mixer.music.play()
    thread_auto = Thread(target = auto_run, args = ())
    thread_auto.start()
    app.run(host='10.0.0.109', port=8090)

    
    
            
        
