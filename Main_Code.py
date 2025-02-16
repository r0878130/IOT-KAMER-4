

import time
import spidev
import wiringpi
import requests  

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

def rotate_stepper_continuous(delay=0.002):
    """Rotate the stepper motor continuously."""
    while stepper_running:
        for step_values in HALF_STEP_SEQUENCE:
            for pin, value in zip(STEPPER_PINS, step_values):
                wiringpi.digitalWrite(pin, value)
            time.sleep(delay)

def stop_stepper():
    """Turn off the stepper motor (set all pins to LOW)."""
    for pin in STEPPER_PINS:
        wiringpi.digitalWrite(pin, 0)

# === STEP 2: Initialize BMP280 Temperature Sensor ===
SPI_BUS = 1
SPI_DEVICE = 0
spi = spidev.SpiDev()
spi.open(SPI_BUS, SPI_DEVICE)
spi.max_speed_hz = 500000

BMP280_REG_CONTROL = 0xF4
BMP280_REG_TEMP = 0xFA
BMP280_REG_ID = 0xD0
BMP280_ID_VALUE = 0x58

def spi_write_register(register, value):
    spi.xfer2([register & 0x7F, value])

def spi_read_register(register):
    return spi.xfer2([register | 0x80, 0x00])[1]

def spi_read_bytes(register, length):
    return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

def init_bmp280():
    chip_id = spi_read_register(BMP280_REG_ID)
    if chip_id != BMP280_ID_VALUE:
        print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
        return None

    spi_write_register(BMP280_REG_CONTROL, 0x27)  
    print("[✅] BMP280 Initialized Successfully")
    return read_calibration_data()

def read_calibration_data():
    calib = spi_read_bytes(0x88, 6)  
    dig_T1 = (calib[1] << 8) | calib[0]
    dig_T2 = (calib[3] << 8) | calib[2]
    dig_T3 = (calib[5] << 8) | calib[4]
    return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

def read_raw_temperature():
    raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
    return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

def compensate_temperature(raw_temp, calib):
    var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
    var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
    t_fine = var1 + var2
    return t_fine / 5120.0  

# === STEP 3: Main Logic ===
desired_temperature = 25.0  
calibration_data = init_bmp280()

BUTTON_INCREASE = 7
BUTTON_DECREASE = 8
wiringpi.pinMode(BUTTON_INCREASE, 0)
wiringpi.pinMode(BUTTON_DECREASE, 0)

if calibration_data:
    try:
        stepper_running = False

        while True:
            if wiringpi.digitalRead(BUTTON_INCREASE) == 1:
                desired_temperature += 1
                print(f"[⬆] Desired Temperature Increased: {desired_temperature}°C")
                time.sleep(0.5)  # Debounce delay

            if wiringpi.digitalRead(BUTTON_DECREASE) == 1:
                desired_temperature -= 1
                print(f"[⬇] Desired Temperature Decreased: {desired_temperature}°C")
                time.sleep(0.5)  

            raw_temp = read_raw_temperature()
            temp_c = compensate_temperature(raw_temp, calibration_data)
            print(f"[🌡] Temperature: {temp_c:.2f} °C | Desired: {desired_temperature:.2f} °C")

            THINGSPEAK_URL_SENSOR = f"https://api.thingspeak.com/update?api_key=LKPW521GXHAU6G31&field4={temp_c:.2f}"
            THINGSPEAK_URL_DESIRED = f"https://api.thingspeak.com/update?api_key=2HBKEUX8ENIXKAGX&field4={desired_temperature:.2f}"
            requests.get(THINGSPEAK_URL_SENSOR)
            requests.get(THINGSPEAK_URL_DESIRED)

            if temp_c > desired_temperature and not stepper_running:
                print("[⚙] Temperature above threshold. Starting stepper motor...")
                stepper_running = True
                rotate_stepper_continuous(delay=0.002)
            elif temp_c <= desired_temperature and stepper_running:
                print("[⚙] Temperature normal. Stopping stepper motor...")
                stepper_running = False
                stop_stepper()

            time.sleep(15)

    except KeyboardInterrupt:
        print("\n[🛑] Exiting program.")
        stepper_running = False
        stop_stepper()
        spi.close()
