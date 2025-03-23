
# #NEW CODE WITH THREADS FOR TEMPERATURE CONTROL! ( ALWAYS USE THREADS IT MAAKE MULTITASKING IN CODE EASY AND PERFECT AND RUNS EVERYTHING SMOOTHLY): 
# import time
# import spidev
# import wiringpi
# import requests
# import threading

# # === Shared Variables ===
# desired_temperature = 25.0  # Shared between threads
# stepper_running = False  # Shared between threads
# lock = threading.Lock()  # Lock for synchronizing access to shared variables

# # === STEP 1: Initialize Stepper Motor ===
# STEPPER_PINS = [3, 4, 5, 6]
# HALF_STEP_SEQUENCE = [
#     [1, 0, 0, 0],
#     [1, 1, 0, 0],
#     [0, 1, 0, 0],
#     [0, 1, 1, 0],
#     [0, 0, 1, 0],
#     [0, 0, 1, 1],
#     [0, 0, 0, 1],
#     [1, 0, 0, 1]
# ]

# wiringpi.wiringPiSetup()
# for pin in STEPPER_PINS:
#     wiringpi.pinMode(pin, 1)

# def rotate_stepper_continuous(delay=0.002):
#     """Rotate the stepper motor continuously."""
#     while True:
#         with lock:
#             if stepper_running:
#                 for step_values in HALF_STEP_SEQUENCE:
#                     for pin, value in zip(STEPPER_PINS, step_values):
#                         wiringpi.digitalWrite(pin, value)
#                     time.sleep(delay)
#             else:
#                 time.sleep(0.1)  # Sleep if stepper is not running

# def stop_stepper():
#     """Turn off the stepper motor (set all pins to LOW)."""
#     for pin in STEPPER_PINS:
#         wiringpi.digitalWrite(pin, 0)

# # === STEP 2: Initialize BMP280 Temperature Sensor ===
# SPI_BUS = 1
# SPI_DEVICE = 0
# spi = spidev.SpiDev()
# spi.open(SPI_BUS, SPI_DEVICE)
# spi.max_speed_hz = 500000

# BMP280_REG_CONTROL = 0xF4
# BMP280_REG_TEMP = 0xFA
# BMP280_REG_ID = 0xD0
# BMP280_ID_VALUE = 0x58

# def spi_write_register(register, value):
#     spi.xfer2([register & 0x7F, value])

# def spi_read_register(register):
#     return spi.xfer2([register | 0x80, 0x00])[1]

# def spi_read_bytes(register, length):
#     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# def init_bmp280():
#     chip_id = spi_read_register(BMP280_REG_ID)
#     if chip_id != BMP280_ID_VALUE:
#         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
#         return None
#     spi_write_register(BMP280_REG_CONTROL, 0x27)
#     print("[✅] BMP280 Initialized Successfully")
#     return read_calibration_data()

# def read_calibration_data():
#     calib = spi_read_bytes(0x88, 6)
#     dig_T1 = (calib[1] << 8) | calib[0]
#     dig_T2 = (calib[3] << 8) | calib[2]
#     dig_T3 = (calib[5] << 8) | calib[4]
#     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# def read_raw_temperature():
#     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
#     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# def compensate_temperature(raw_temp, calib):
#     var1 = ((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"]
#     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
#     t_fine = var1 + var2
#     return t_fine / 5120.0

# # === STEP 3: Thread Functions ===
# def temperature_monitoring_thread():
#     global stepper_running
#     calibration_data = init_bmp280()
#     if not calibration_data:
#         return

#     while True:
#         raw_temp = read_raw_temperature()
#         temp_c = compensate_temperature(raw_temp, calibration_data)
#         print(f"[🌡] Temperature: {temp_c:.2f} °C | Desired: {desired_temperature:.2f} °C")

#         # Send data to ThingSpeak
#         THINGSPEAK_URL_SENSOR = f"https://api.thingspeak.com/update?api_key=LKPW521GXHAU6G31&field4={temp_c:.2f}"
#         THINGSPEAK_URL_DESIRED = f"https://api.thingspeak.com/update?api_key=2HBKEUX8ENIXKAGX&field4={desired_temperature:.2f}"
#         requests.get(THINGSPEAK_URL_SENSOR)
#         requests.get(THINGSPEAK_URL_DESIRED)

#         # Control stepper motor
#         with lock:
#             if temp_c > desired_temperature and not stepper_running:
#                 print("[⚙] Temperature above threshold. Starting stepper motor...")
#                 stepper_running = True
#             elif temp_c <= desired_temperature and stepper_running:
#                 print("[⚙] Temperature normal. Stopping stepper motor...")
#                 stepper_running = False
#                 stop_stepper()

#         time.sleep(15)  # Adjust sleep time as needed

# def button_handling_thread():
#     global desired_temperature
#     BUTTON_INCREASE = 7
#     BUTTON_DECREASE = 8
#     wiringpi.pinMode(BUTTON_INCREASE, wiringpi.INPUT)
#     wiringpi.pullUpDnControl(BUTTON_INCREASE, wiringpi.PUD_UP)
#     wiringpi.pinMode(BUTTON_DECREASE, wiringpi.INPUT)
#     wiringpi.pullUpDnControl(BUTTON_DECREASE, wiringpi.PUD_UP)

#     prev_increase_state = wiringpi.digitalRead(BUTTON_INCREASE)
#     prev_decrease_state = wiringpi.digitalRead(BUTTON_DECREASE)
#     DEBOUNCE_DELAY = 0.5

#     while True:
#         increase_state = wiringpi.digitalRead(BUTTON_INCREASE)
#         decrease_state = wiringpi.digitalRead(BUTTON_DECREASE)

#         if increase_state == wiringpi.LOW and prev_increase_state == wiringpi.HIGH:
#             with lock:
#                 desired_temperature += 1
#                 print(f"[⬆] Desired Temperature Increased: {desired_temperature}°C")
#             time.sleep(DEBOUNCE_DELAY)

#         if decrease_state == wiringpi.LOW and prev_decrease_state == wiringpi.HIGH:
#             with lock:
#                 desired_temperature -= 1
#                 print(f"[⬇] Desired Temperature Decreased: {desired_temperature}°C")
#             time.sleep(DEBOUNCE_DELAY)

#         prev_increase_state = increase_state
#         prev_decrease_state = decrease_state
#         time.sleep(0.1)

# # === STEP 4: Start Threads ===
# if __name__ == "__main__":
#     # Create threads
#     temp_thread = threading.Thread(target=temperature_monitoring_thread)
#     button_thread = threading.Thread(target=button_handling_thread)
#     stepper_thread = threading.Thread(target=rotate_stepper_continuous)

#     # Start threads
#     temp_thread.start()
#     button_thread.start()
#     stepper_thread.start()

#     # Wait for threads to finish (they won't in this case)
#     temp_thread.join()
#     button_thread.join()
#     stepper_thread.join()








# #NEW CODE WITH THREADS FOR LIGHT/LUX CONTROL! ( ALWAYS USE THREADS IT MAAKE MULTITASKING IN CODE EASY AND PERFECT AND RUNS EVERYTHING SMOOTHLY): 
import smbus
import time
import threading
import requests
import wiringpi

# === Shared Variables ===
desired_lux = 100.0  # Shared between threads
stepper_running = False  # Shared between threads
lock = threading.Lock()  # Lock for synchronizing access to shared variables
curtains_open = False  # Track curtain state

# === STEP 1: Initialize Stepper Motor ===
STEPPER_PINS = [3, 4, 5, 6]
HALF_STEP_SEQUENCE = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

wiringpi.wiringPiSetup()
for pin in STEPPER_PINS:
    wiringpi.pinMode(pin, 1)

def rotate_stepper_clockwise(delay=0.002):
    """Rotate the stepper motor clockwise (open curtains)."""
    while True:
        with lock:
            if stepper_running == "clockwise":
                for step_values in HALF_STEP_SEQUENCE:
                    for pin, value in zip(STEPPER_PINS, step_values):
                        wiringpi.digitalWrite(pin, value)
                    time.sleep(delay)
            else:
                time.sleep(0.1)  # Sleep if stepper is not running clockwise

def rotate_stepper_anticlockwise(delay=0.002):
    """Rotate the stepper motor anticlockwise (close curtains)."""
    while True:
        with lock:
            if stepper_running == "anticlockwise":
                for step_values in reversed(HALF_STEP_SEQUENCE):
                    for pin, value in zip(STEPPER_PINS, step_values):
                        wiringpi.digitalWrite(pin, value)
                    time.sleep(delay)
            else:
                time.sleep(0.1)  # Sleep if stepper is not running anticlockwise

def stop_stepper():
    """Turn off the stepper motor (set all pins to LOW)."""
    for pin in STEPPER_PINS:
        wiringpi.digitalWrite(pin, 0)

# === STEP 2: Initialize BH1750 Light Sensor ===
I2C_BUS = 0  # Use bus 0 on Orange Pi
BH1750_ADDR = 0x23
bus = smbus.SMBus(I2C_BUS)

def read_bh1750():
    try:
        data = bus.read_i2c_block_data(BH1750_ADDR, 0x20, 2)
        lux = (data[0] << 8) + data[1]
        return lux / 1.2  # Convert to lux
    except IOError:
        print("[⚠] Error reading from BH1750 sensor.")
        return None

# === STEP 3: Thread Functions ===
def lux_monitoring_thread():
    global stepper_running, curtains_open
    start_time = time.time()  # Track when the motor starts running

    while True:
        measured_lux = read_bh1750()
        if measured_lux is not None:
            print(f"[💡] Measured Lux: {measured_lux:.2f} | Desired Lux: {desired_lux:.2f}")

            # Send data to ThingSpeak
            THINGSPEAK_URL_SENSOR = f"https://api.thingspeak.com/update?api_key=LKPW521GXHAU6G31&field4={measured_lux:.2f}"
            THINGSPEAK_URL_DESIRED = f"https://api.thingspeak.com/update?api_key=2HBKEUX8ENIXKAGX&field4={desired_lux:.2f}"
            requests.get(THINGSPEAK_URL_SENSOR)
            requests.get(THINGSPEAK_URL_DESIRED)

            # Control stepper motor
            with lock:
                if measured_lux < desired_lux and not curtains_open:
                    print("[⚙] Lux is LOW. Opening curtains (clockwise)...")
                    stepper_running = "clockwise"
                    curtains_open = True
                    start_time = time.time()  # Reset the timer
                elif measured_lux >= desired_lux and curtains_open:
                    print("[⚙] Lux is HIGH. Stopping curtains...")
                    stepper_running = False
                    curtains_open = False
                    stop_stepper()
                elif stepper_running == "clockwise" and (time.time() - start_time) >= 300:  # 5 minutes
                    print("[🌙] Lux not achieved in 5 minutes. Closing curtains (anticlockwise)...")
                    stepper_running = "anticlockwise"
                    curtains_open = False
                    start_time = time.time()  # Reset the timer

        time.sleep(2)  # Check every 2 seconds

def button_handling_thread():
    global desired_lux
    BUTTON_INCREASE = 7
    BUTTON_DECREASE = 8
    wiringpi.pinMode(BUTTON_INCREASE, wiringpi.INPUT)
    wiringpi.pullUpDnControl(BUTTON_INCREASE, wiringpi.PUD_UP)
    wiringpi.pinMode(BUTTON_DECREASE, wiringpi.INPUT)
    wiringpi.pullUpDnControl(BUTTON_DECREASE, wiringpi.PUD_UP)

    prev_increase_state = wiringpi.digitalRead(BUTTON_INCREASE)
    prev_decrease_state = wiringpi.digitalRead(BUTTON_DECREASE)
    DEBOUNCE_DELAY = 0.5

    while True:
        increase_state = wiringpi.digitalRead(BUTTON_INCREASE)
        decrease_state = wiringpi.digitalRead(BUTTON_DECREASE)

        if increase_state == wiringpi.LOW and prev_increase_state == wiringpi.HIGH:
            with lock:
                desired_lux += 10
                print(f"[⬆] Desired Lux Increased: {desired_lux}")
            time.sleep(DEBOUNCE_DELAY)

        if decrease_state == wiringpi.LOW and prev_decrease_state == wiringpi.HIGH:
            with lock:
                desired_lux -= 10
                print(f"[⬇] Desired Lux Decreased: {desired_lux}")
            time.sleep(DEBOUNCE_DELAY)

        prev_increase_state = increase_state
        prev_decrease_state = decrease_state
        time.sleep(0.1)

# === STEP 4: Start Threads ===
if __name__ == "__main__":
    # Create threads
    lux_thread = threading.Thread(target=lux_monitoring_thread)
    button_thread = threading.Thread(target=button_handling_thread)
    stepper_clockwise_thread = threading.Thread(target=rotate_stepper_clockwise)
    stepper_anticlockwise_thread = threading.Thread(target=rotate_stepper_anticlockwise)

    # Start threads
    lux_thread.start()
    button_thread.start()
    stepper_clockwise_thread.start()
    stepper_anticlockwise_thread.start()

    # Wait for threads to finish (they won't in this case)
    lux_thread.join()
    button_thread.join()
    stepper_clockwise_thread.join()
    stepper_anticlockwise_thread.join()
