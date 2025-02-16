# # # #STEPPER MOTOR + DRIVER BOARD SEPERATE

# # # import time
# # # import wiringpi

# # # # Define GPIO Pins (Check your wiring)
# # # STEPPER_PINS = [3, 4, 5, 6]  # ULN2003 IN1, IN2, IN3, IN4

# # # # Initialize WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper pins as OUTPUT
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # # *Fastest Half-Step Sequence for Max Speed*
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Function to rotate stepper motor at full speed
# # # def rotate_stepper_fast(steps=1024, delay=0.002):
# # #     """
# # #     Rotates the stepper motor at maximum speed.
# # #     - steps: Number of steps to move
# # #     - delay: Minimum delay for fastest speed (Default: 0.002s)
# # #     """
# # #     print(f"\n[INFO] Running Stepper for {steps} Steps at Max Speed (Delay={delay}s)\n")
    
# # #     for step in range(steps):
# # #         for step_number, step_values in enumerate(HALF_STEP_SEQUENCE):
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)  # Minimum delay for full speed

# # #     print("\n[INFO] Stepper Reached Full Rotation!\n")

# # # # *Run Stepper Motor at Maximum Speed*
# # # print("\n[TEST] Running Stepper at Full Speed...")
# # # rotate_stepper_fast(steps=4096, delay=0.002)  # 4096 steps = ~5 full rotations

# # # # Cleanup: Turn off stepper motor
# # # for pin in STEPPER_PINS:
# # #     wiringpi.digitalWrite(pin, 0)

# # # print("\n=== Stepper Motor Test Complete ===")






# # # # ULTRA SONIC SEPERATE 

# # # # import time
# # # # import wiringpi

# # # # # Define GPIO pins (Updated with new pins)
# # # # TRIG_PIN = 16  # w16 (Pin 16)
# # # # ECHO_PIN = 13  # w5 (Pin 13)

# # # # # Initialize WiringPi
# # # # wiringpi.wiringPiSetup()

# # # # # Set GPIO modes
# # # # wiringpi.pinMode(TRIG_PIN, 1)  # TRIG as OUTPUT
# # # # wiringpi.pinMode(ECHO_PIN, 0)  # ECHO as INPUT

# # # # # Function to test the ultrasonic sensor
# # # # def measure_distance():
# # # #     print("\n[INFO] Sending Trigger Signal...")

# # # #     # Send Trigger Pulse
# # # #     wiringpi.digitalWrite(TRIG_PIN, 1)
# # # #     time.sleep(0.00001)  # 10 microsecond pulse
# # # #     wiringpi.digitalWrite(TRIG_PIN, 0)

# # # #     # Wait for echo start
# # # #     start_time = time.time()
# # # #     timeout_start = start_time + 0.05  # 50ms timeout
# # # #     while wiringpi.digitalRead(ECHO_PIN) == 0:
# # # #         start_time = time.time()
# # # #         if start_time > timeout_start:
# # # #             print("[⚠] ECHO Start Timeout! Check wiring.")
# # # #             return None

# # # #     # Wait for echo end
# # # #     stop_time = time.time()
# # # #     timeout_stop = stop_time + 0.05  # 50ms timeout
# # # #     while wiringpi.digitalRead(ECHO_PIN) == 1:
# # # #         stop_time = time.time()
# # # #         if stop_time > timeout_stop:
# # # #             print("[⚠] ECHO End Timeout! Check wiring.")
# # # #             return None

# # # #     # Calculate time difference
# # # #     time_elapsed = stop_time - start_time

# # # #     # Convert to distance (Speed of sound = 34300 cm/s)
# # # #     distance = (time_elapsed * 34300) / 2  # Divide by 2 for round-trip

# # # #     return round(distance, 2)  # Return distance in cm

# # # # # Continuous measurement loop
# # # # print("\n[INFO] Running Ultrasonic Distance Test...")
# # # # try:
# # # #     while True:
# # # #         distance = measure_distance()

# # # #         if distance is not None:
# # # #             print(f"[📏] Distance: {distance} cm")
# # # #         else:
# # # #             print("[⚠] No valid reading, check wiring!")

# # # #         time.sleep(1)  # Delay before next reading

# # # # except KeyboardInterrupt:
# # # #     print("\n[INFO] Stopping Ultrasonic Sensor Test...")
# # # #     print("\n=== HC-SR04 Sensor Test Stopped ===")


# # # # STEEPER MOTOR + DRIVER BOARD + ULTRASONIC MOTOR ALL TOGETHER !!! 

# # # import time
# # # import wiringpi
# # # import threading

# # # # === Stepper Motor Configuration ===
# # # STEPPER_PINS = [3, 4, 6, 9]  # ULN2003 IN1, IN2, IN3, IN4

# # # # Initialize WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper pins as OUTPUT
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # # Fastest Half-Step Sequence
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Function to rotate stepper motor at full speed
# # # def rotate_stepper_fast(steps=4096, delay=0.001):
# # #     """
# # #     Rotates the stepper motor at maximum speed.
# # #     - steps: Number of steps to move (~5 full rotations)
# # #     - delay: Minimum delay for fastest speed (Default: 0.002s)
# # #     """
# # #     print(f"\n[INFO] Running Stepper for {steps} Steps at Max Speed (Delay={delay}s)\n")
    
# # #     for step in range(steps):
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)  # Minimum delay for full speed

# # #     print("\n[INFO] Stepper Reached Full Rotation!\n")




# # # # === Ultrasonic Sensor Configuration ===
# # # TRIG_PIN = 16  # w16 (Pin 16)
# # # ECHO_PIN = 13  # w5 (Pin 13)

# # # # Set GPIO modes
# # # wiringpi.pinMode(TRIG_PIN, 1)  # TRIG as OUTPUT
# # # wiringpi.pinMode(ECHO_PIN, 0)  # ECHO as INPUT

# # # # Function to measure distance
# # # def measure_distance():
# # #     """
# # #     Measures distance using HC-SR04 ultrasonic sensor.
# # #     Returns distance in cm or None if there's an issue.
# # #     """
# # #     print("\n[INFO] Sending Trigger Signal...")

# # #     # Send Trigger Pulse
# # #     wiringpi.digitalWrite(TRIG_PIN, 1)
# # #     time.sleep(0.00001)  # 10-microsecond pulse
# # #     wiringpi.digitalWrite(TRIG_PIN, 0)

# # #     # Wait for echo start
# # #     start_time = time.time()
# # #     timeout_start = start_time + 0.05  # 50ms timeout
# # #     while wiringpi.digitalRead(ECHO_PIN) == 0:
# # #         start_time = time.time()
# # #         if start_time > timeout_start:
# # #             print("[⚠] ECHO Start Timeout! Check wiring.")
# # #             return None

# # #     # Wait for echo end
# # #     stop_time = time.time()
# # #     timeout_stop = stop_time + 0.05  # 50ms timeout
# # #     while wiringpi.digitalRead(ECHO_PIN) == 1:
# # #         stop_time = time.time()
# # #         if stop_time > timeout_stop:
# # #             print("[⚠] ECHO End Timeout! Check wiring.")
# # #             return None

# # #     # Calculate time difference
# # #     time_elapsed = stop_time - start_time

# # #     # Convert to distance (Speed of sound = 34300 cm/s)
# # #     distance = (time_elapsed * 34300) / 2  # Divide by 2 for round-trip

# # #     return round(distance, 2)  # Return distance in cm

# # # # Function to run ultrasonic sensor continuously
# # # def run_ultrasonic():
# # #     """
# # #     Continuously measures distance and prints it.
# # #     """
# # #     print("\n[INFO] Running Ultrasonic Distance Test...")
# # #     try:
# # #         while True:
# # #             distance = measure_distance()
# # #             if distance is not None:
# # #                 print(f"[📏] Distance: {distance} cm")
# # #             else:
# # #                 print("[⚠] No valid reading, check wiring!")

# # #             time.sleep(1)  # Delay before next reading
# # #     except KeyboardInterrupt:
# # #         print("\n[INFO] Stopping Ultrasonic Sensor Test...")
# # #         print("\n=== HC-SR04 Sensor Test Stopped ===")

# # # # === Running Both Processes Simultaneously ===
# # # if __name__ == "__main__":
# # #     # Create separate threads for stepper and ultrasonic sensor
# # #     stepper_thread = threading.Thread(target=rotate_stepper_fast, args=(4096, 0.002))
# # #     ultrasonic_thread = threading.Thread(target=run_ultrasonic)

# # #     # Start both threads
# # #     stepper_thread.start()
# # #     ultrasonic_thread.start()

# # #     # Wait for both threads to finish
# # #     stepper_thread.join()
# # #     ultrasonic_thread.join()

# # #     # Cleanup: Turn off stepper motor
# # #     for pin in STEPPER_PINS:
# # #         wiringpi.digitalWrite(pin, 0)

# # #     print("\n=== Stepper Motor & Ultrasonic Sensor Test Complete ===")



# # #import sys
# # #import time
# # #from smbus2 import SMBus
# # #from bmp280 import BMP280  # Adjusted import

# # # Add library path if necessary
# # #sys.path.append('/home/orangepi/.local/lib/python3.9/site-packages')

# # # Initialize I2C bus
# # #bus = SMBus(0)  # Try 1 or 2 if 0 does not work

# # # Initialize BMP280 sensor
# # #bmp = BMP280(i2c_addr=0x77, i2c_dev=bus)  # Change to 0x76 if needed

# # #print("\n[INFO] BMP280 Sensor Test...\n")

# # #try:
# # #    while True:
# # #        temperature = bmp.get_temperature()
# # #        pressure = bmp.get_pressure()
# # #        print(f"[🌡] Temperature: {temperature:.2f}°C | [⏲] Pressure: {pressure:.2f} hPa")
# # #        time.sleep(1)  # Read every second

# # #except KeyboardInterrupt:
# # #    print("\n[INFO] BMP280 Test Stopped.")



# # # import time
# # # import spidev

# # # # Set up SPI
# # # spi = spidev.SpiDev()
# # # spi.open(1, 0)  # Open SPI bus 1, device 0
# # # spi.max_speed_hz = 50000

# # # # Initialize BMP280 (check datasheet for specific initialization commands)
# # # try:
# # #     while True:
# # #         # Read data (simplified for this example)
# # #         print("Reading BMP280 via SPI...")
# # #         time.sleep(1)
# # # except KeyboardInterrupt:
# # #     print("Exiting BMP280 SPI test.")




# # # # pressure and temprature
# # # import time
# # # import spidev

# # # # *SPI Configuration*
# # # SPI_BUS = 1  # SPI Bus 1
# # # SPI_DEVICE = 0  # CE0 (Chip Enable 0)
# # # spi = spidev.SpiDev()
# # # spi.open(SPI_BUS, SPI_DEVICE)
# # # spi.max_speed_hz = 500000  # SPI speed: 500 kHz

# # # # *BMP280 Registers*
# # # BMP280_REG_CONTROL = 0xF4
# # # BMP280_REG_TEMP = 0xFA
# # # BMP280_REG_PRESS = 0xF7
# # # BMP280_REG_CONFIG = 0xF5
# # # BMP280_REG_ID = 0xD0
# # # BMP280_ID_VALUE = 0x58  # Expected ID for BMP280

# # # # *Read Calibration Registers*
# # # def read_calibration_data():
# # #     """Reads BMP280 factory calibration data for accurate compensation"""
# # #     calib = spi_read_bytes(0x88, 24)  # Calibration registers start at 0x88
# # #     dig_T1 = (calib[1] << 8) | calib[0]
# # #     dig_T2 = (calib[3] << 8) | calib[2]
# # #     dig_T3 = (calib[5] << 8) | calib[4]
# # #     dig_P1 = (calib[7] << 8) | calib[6]
# # #     dig_P2 = (calib[9] << 8) | calib[8]
# # #     dig_P3 = (calib[11] << 8) | calib[10]
# # #     dig_P4 = (calib[13] << 8) | calib[12]
# # #     dig_P5 = (calib[15] << 8) | calib[14]
# # #     dig_P6 = (calib[17] << 8) | calib[16]
# # #     dig_P7 = (calib[19] << 8) | calib[18]
# # #     dig_P8 = (calib[21] << 8) | calib[20]
# # #     dig_P9 = (calib[23] << 8) | calib[22]

# # #     return {
# # #         "T1": dig_T1, "T2": dig_T2, "T3": dig_T3,
# # #         "P1": dig_P1, "P2": dig_P2, "P3": dig_P3,
# # #         "P4": dig_P4, "P5": dig_P5, "P6": dig_P6,
# # #         "P7": dig_P7, "P8": dig_P8, "P9": dig_P9
# # #     }

# # # # *SPI Read/Write Helper Functions*
# # # def spi_write_register(register, value):
# # #     """Write a byte to BMP280 register."""
# # #     spi.xfer2([register & 0x7F, value])

# # # def spi_read_register(register):
# # #     """Read a single byte from BMP280."""
# # #     return spi.xfer2([register | 0x80, 0x00])[1]

# # # def spi_read_bytes(register, length):
# # #     """Read multiple bytes from BMP280."""
# # #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # # # *BMP280 Initialization*
# # # def init_bmp280():
# # #     """Initialize the BMP280 sensor with default settings."""
# # #     chip_id = spi_read_register(BMP280_REG_ID)
# # #     if chip_id != BMP280_ID_VALUE:
# # #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# # #         return None

# # #     # Set control register: Normal mode, 1x oversampling for Temp & Pressure
# # #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# # #     # Set config register: Standby time = 1000ms, IIR filter off
# # #     spi_write_register(BMP280_REG_CONFIG, 0xA0)

# # #     print("[✅] BMP280 Initialized Successfully")
# # #     return read_calibration_data()

# # # # *Read Raw Data*
# # # def read_raw_temperature():
# # #     """Read raw temperature data from BMP280."""
# # #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # def read_raw_pressure():
# # #     """Read raw pressure data from BMP280."""
# # #     raw_data = spi_read_bytes(BMP280_REG_PRESS, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # # *Compensation Formulas*
# # # def compensate_temperature(raw_temp, calib):
# # #     """Convert raw temperature to Celsius using BMP280 calibration."""
# # #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# # #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# # #     t_fine = var1 + var2
# # #     return t_fine / 5120.0  # Convert to Celsius

# # # def compensate_pressure(raw_press, calib, t_fine):
# # #     """Convert raw pressure to hPa using BMP280 calibration."""
# # #     var1 = (t_fine / 2.0) - 64000.0
# # #     var2 = var1 * var1 * calib["P6"] / 32768.0
# # #     var2 = var2 + var1 * calib["P5"] * 2.0
# # #     var2 = (var2 / 4.0) + (calib["P4"] * 65536.0)
# # #     var1 = ((calib["P3"] * var1 * var1 / 524288.0) + (calib["P2"] * var1)) / 524288.0
# # #     var1 = (1.0 + var1 / 32768.0) * calib["P1"]
    
# # #     if var1 == 0:
# # #         return 0  # Avoid division by zero

# # #     pressure = 1048576.0 - raw_press
# # #     pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1
# # #     var1 = calib["P9"] * pressure * pressure / 2147483648.0
# # #     var2 = pressure * calib["P8"] / 32768.0
# # #     pressure = pressure + (var1 + var2 + calib["P7"]) / 16.0

# # #     return pressure / 100.0  # Convert to hPa

# # # # *Main Loop*
# # # calibration_data = init_bmp280()
# # # if calibration_data:
# # #     try:
# # #         while True:
# # #             raw_temp = read_raw_temperature()
# # #             raw_press = read_raw_pressure()

# # #             temp_c = compensate_temperature(raw_temp, calibration_data)
# # #             press_hpa = compensate_pressure(raw_press, calibration_data, temp_c)

# # #             print(f"[🌡] Temperature: {temp_c:.2f} °C | [⏲] Pressure: {press_hpa:.2f} hPa")
# # #             time.sleep(1)

# # #     except KeyboardInterrupt:
# # #         print("\n[🛑] Exiting BMP280 Sensor Test.")
# # #         spi.close()




# # # # Lux by light intensity

# # # import smbus
# # # import time

# # # # Initialize the I2C bus 0
# # # bus = smbus.SMBus(0)  # Use bus 0 instead of 1

# # # # BH1750 address
# # # address = 0x23

# # # # Function to read data from the sensor
# # # def read_bh1750():
# # #     # Read data from the BH1750 sensor (simplified)
# # #     try:
# # #         data = bus.read_i2c_block_data(address, 0x20, 2)  # 0x20 is the command for continuous high res mode
# # #         lux = (data[0] << 8) + data[1]
# # #         lux = lux / 1.2  # Convert to lux
# # #         print(f"Ambient Light: {lux} lux")
# # #     except IOError:
# # #         print("Error reading from sensor.")

# # # # Main loop
# # # try:
# # #     while True:
# # #         read_bh1750()
# # #         time.sleep(2)
# # # except KeyboardInterrupt:
# # #     print("Exiting program.")



# # # # Controlling the LED on driver board using the Pwm ! 

# # # import time
# # # import wiringpi as wpi
# # # import sys

# # # def controlLEDs(sig1, sig2, cnt, wait):
# # #     wpi.softPwmWrite(sig1, cnt)
# # #     wpi.softPwmWrite(sig2, 100 - cnt)
# # #     time.sleep(wait)

# # # # SETUP
# # # print("Start")
# # # pin2 = 2
# # # pin5 = 5
# # # pause_time = 0.82
# # # wpi.wiringPiSetup()

# # # # Set pins as a softPWM output
# # # wpi.softPwmCreate(pin2, 0, 100)
# # # wpi.softPwmCreate(pin5, 0, 100)

# # # # Start PWM
# # # wpi.softPwmWrite(pin2, 0)
# # # wpi.softPwmWrite(pin5, 100)

# # # try:
# # #     while True:
# # #         for i in range(0, 101):  # 101 because it stops when it finishes 100
# # #             controlLEDs(pin2, pin5, i, pause_time)
# # #         for i in range(100, -1, -1):  # from 100 to zero in steps of -1
# # #             controlLEDs(pin2, pin5, i, pause_time)

# # # except KeyboardInterrupt:
# # #     wpi.softPwmWrite(pin2, 0)  # stop the white PWM output
# # #     wpi.softPwmWrite(pin5, 0)  # stop the white PWM output
# # #     print("\nDone")





# # # # Stepper motor + light sensor for clockwise and anticlockwise rotation of stepper motor for the curtains roll up :

# # # import time
# # # import wiringpi
# # # import smbus

# # # # BH1750 Sensor Initialization
# # # bus = smbus.SMBus(0)  # Initialize the I2C bus 0
# # # BH1750_ADDRESS = 0x23  # Address of BH1750 sensor
# # # LUX_THRESHOLD = 20  # Lux threshold for curtain control

# # # # Stepper Motor Initialization
# # # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Setup WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper motor pins as output
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # # Function to read light intensity from BH1750 sensor
# # # def read_bh1750():
# # #     try:
# # #         # Read 2 bytes of data from BH1750 (High-Res Mode)
# # #         data = bus.read_i2c_block_data(BH1750_ADDRESS, 0x20, 2)
# # #         lux = (data[0] << 8) + data[1]
# # #         lux = lux / 1.2  # Convert raw data to lux
# # #         return lux
# # #     except IOError:
# # #         print("Error reading from BH1750 sensor.")
# # #         return None

# # # # Function to rotate stepper motor clockwise
# # # def rotate_stepper_clockwise(steps=512, delay=0.002):
# # #     for step in range(steps):
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # # Function to rotate stepper motor anti-clockwise
# # # def rotate_stepper_anticlockwise(steps=512, delay=0.002):
# # #     for step in range(steps):
# # #         for step_values in reversed(HALF_STEP_SEQUENCE):  # Reverse the sequence for anti-clockwise motion
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # # Function to control curtains based on light intensity
# # # def control_curtains():
# # #     lux = read_bh1750()  # Read the current lux value from the sensor
# # #     if lux is None:
# # #         return  # Skip if there's an error reading from the sensor

# # #     print(f"Current Light Intensity: {lux} lux")

# # #     if lux > LUX_THRESHOLD:
# # #         print("Lux is greater than threshold. Rolling curtains to close (Clockwise).")
# # #         rotate_stepper_clockwise(steps=1024, delay=0.002)  # Adjust steps for 20 seconds of motion
# # #     elif lux <= LUX_THRESHOLD:
# # #         print("Lux is less than or equal to threshold. Rolling curtains to open (Anti-clockwise).")
# # #         rotate_stepper_anticlockwise(steps=1024, delay=0.002)  # Adjust steps for 20 seconds of motion

# # # # Main program loop
# # # try:
# # #     print("Starting automatic curtain control...")
# # #     while True:
# # #         control_curtains()
# # #         time.sleep(20)  # Check light intensity every 20 seconds
# # # except KeyboardInterrupt:
# # #     print("Exiting program. Turning off stepper motor.")

# # # # Cleanup: Turn off stepper motor
# # # for pin in STEPPER_PINS:
# # #     wiringpi.digitalWrite(pin, 0)




# # # #new updated code Use case light sensor with rolling and unrolling curtains ! 
# # # import time
# # # import wiringpi
# # # import smbus

# # # # BH1750 Sensor Initialization
# # # bus = smbus.SMBus(0)  # Initialize the I2C bus 0
# # # BH1750_ADDRESS = 0x23  # Address of BH1750 sensor
# # # LUX_THRESHOLD = 20  # Lux threshold for curtain control

# # # # Stepper Motor Initialization
# # # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Setup WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper motor pins as output
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # # Function to read light intensity from BH1750 sensor
# # # def read_bh1750():
# # #     try:
# # #         # Read 2 bytes of data from BH1750 (High-Res Mode)
# # #         data = bus.read_i2c_block_data(BH1750_ADDRESS, 0x20, 2)
# # #         lux = (data[0] << 8) + data[1]
# # #         lux = lux / 1.2  # Convert raw data to lux
# # #         return lux
# # #     except IOError:
# # #         print("Error reading from BH1750 sensor.")
# # #         return None

# # # # Function to rotate stepper motor clockwise
# # # def rotate_stepper_clockwise(duration=300, delay=0.002):
# # #     steps_per_cycle = len(HALF_STEP_SEQUENCE)
# # #     total_steps = int(duration / (delay * steps_per_cycle))  # Calculate total steps for given duration
# # #     for step in range(total_steps):
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # # Function to rotate stepper motor anti-clockwise
# # # def rotate_stepper_anticlockwise(duration=300, delay=0.002):
# # #     steps_per_cycle = len(HALF_STEP_SEQUENCE)
# # #     total_steps = int(duration / (delay * steps_per_cycle))  # Calculate total steps for given duration
# # #     for step in range(total_steps):
# # #         for step_values in reversed(HALF_STEP_SEQUENCE):  # Reverse the sequence for anti-clockwise motion
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # # Function to control curtains based on light intensity
# # # def control_curtains():
# # #     global last_action  # Track last action to avoid repeated rotations
# # #     lux = read_bh1750()  # Read the current lux value from the sensor
# # #     if lux is None:
# # #         return  # Skip if there's an error reading from the sensor

# # #     print(f"Current Light Intensity: {lux} lux")

# # #     if lux > LUX_THRESHOLD and last_action != "close":
# # #         print("Lux is greater than threshold. Rolling curtains to open (Clockwise).")
# # #         rotate_stepper_clockwise(duration=300, delay=0.002)  # Rotate for 5 minutes
# # #         last_action = "close"
# # #     elif lux <= LUX_THRESHOLD and last_action != "open":
# # #         print("Lux is less than or equal to threshold. Rolling curtains to close (Anti-clockwise).")
# # #         rotate_stepper_anticlockwise(duration=300, delay=0.002)  # Rotate for 5 minutes
# # #         last_action = "open"

# # # # Initialize the last action
# # # last_action = None

# # # # Main program loop
# # # try:
# # #     print("Starting automatic curtain control...")
# # #     while True:
# # #         control_curtains()
# # #         time.sleep(20)  # Check light intensity every 20 seconds
# # # except KeyboardInterrupt:
# # #     print("Exiting program. Turning off stepper motor.")

# # # # Cleanup: Turn off stepper motor
# # # for pin in STEPPER_PINS:
# # #     wiringpi.digitalWrite(pin, 0)
 




# # # #Temperature and stepper motor code 

# # # import time
# # # import spidev
# # # import wiringpi

# # # # === STEP 1: Initialize Stepper Motor ===
# # # STEPPER_PINS = [3, 4, 5, 6]  # ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Initialize WiringPi
# # # wiringpi.wiringPiSetup()
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # def rotate_stepper_fast(steps=1024, delay=0.002):
# # #     """Rotate the stepper motor at maximum speed."""
# # #     for step in range(steps):
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)  # Small delay for smooth rotation

# # # def stop_stepper():
# # #     """Turn off the stepper motor (set all pins to LOW)."""
# # #     for pin in STEPPER_PINS:
# # #         wiringpi.digitalWrite(pin, 0)

# # # # === STEP 2: Initialize BMP280 Sensor ===
# # # SPI_BUS = 1
# # # SPI_DEVICE = 0
# # # spi = spidev.SpiDev()
# # # spi.open(SPI_BUS, SPI_DEVICE)
# # # spi.max_speed_hz = 500000

# # # BMP280_REG_CONTROL = 0xF4
# # # BMP280_REG_TEMP = 0xFA
# # # BMP280_REG_PRESS = 0xF7
# # # BMP280_REG_CONFIG = 0xF5
# # # BMP280_REG_ID = 0xD0
# # # BMP280_ID_VALUE = 0x58

# # # def spi_write_register(register, value):
# # #     spi.xfer2([register & 0x7F, value])

# # # def spi_read_register(register):
# # #     return spi.xfer2([register | 0x80, 0x00])[1]

# # # def spi_read_bytes(register, length):
# # #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # # def init_bmp280():
# # #     chip_id = spi_read_register(BMP280_REG_ID)
# # #     if chip_id != BMP280_ID_VALUE:
# # #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# # #         return None

# # #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# # #     spi_write_register(BMP280_REG_CONFIG, 0xA0)
# # #     print("[✅] BMP280 Initialized Successfully")
# # #     return read_calibration_data()

# # # def read_calibration_data():
# # #     calib = spi_read_bytes(0x88, 24)
# # #     dig_T1 = (calib[1] << 8) | calib[0]
# # #     dig_T2 = (calib[3] << 8) | calib[2]
# # #     dig_T3 = (calib[5] << 8) | calib[4]
# # #     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# # # def read_raw_temperature():
# # #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # def compensate_temperature(raw_temp, calib):
# # #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# # #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# # #     t_fine = var1 + var2
# # #     return t_fine / 5120.0  # Celsius

# # # # === STEP 3: Main Logic ===
# # # TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# # # calibration_data = init_bmp280()

# # # if calibration_data:
# # #     try:
# # #         stepper_running = False

# # #         while True:
# # #             # Read temperature
# # #             raw_temp = read_raw_temperature()
# # #             temp_c = compensate_temperature(raw_temp, calibration_data)
# # #             print(f"[🌡] Temperature: {temp_c:.2f} °C")

# # #             # Control the stepper motor based on temperature
# # #             if temp_c > TEMP_THRESHOLD and not stepper_running:
# # #                 print("[⚙] Temperature is above threshold. Starting stepper motor...")
# # #                 stepper_running = True
# # #                 rotate_stepper_fast(steps=1024, delay=0.002)  # Rotate stepper motor
# # #             elif temp_c <= TEMP_THRESHOLD and stepper_running:
# # #                 print("[⚙] Temperature is back to normal. Stopping stepper motor...")
# # #                 stepper_running = False
# # #                 stop_stepper()

# # #             time.sleep(1)  # Check temperature every second

# # #     except KeyboardInterrupt:
# # #         print("\n[🛑] Exiting program.")
# # #         stop_stepper()
# # #         spi.close()



# # # import time
# # # import spidev
# # # import wiringpi

# # # # === STEP 1: Initialize Stepper Motor ===
# # # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Setup WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper motor pins as output
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # # Function to rotate stepper motor continuously
# # # def rotate_stepper_continuous(delay=0.002):
# # #     """Rotate the stepper motor continuously."""
# # #     while stepper_running:  # Keep rotating as long as stepper_running is True
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # # Function to stop the stepper motor
# # # def stop_stepper():
# # #     """Turn off the stepper motor (set all pins to LOW)."""
# # #     for pin in STEPPER_PINS:
# # #         wiringpi.digitalWrite(pin, 0)

# # # # === STEP 2: Initialize BMP280 Sensor ===
# # # SPI_BUS = 1
# # # SPI_DEVICE = 0
# # # spi = spidev.SpiDev()
# # # spi.open(SPI_BUS, SPI_DEVICE)
# # # spi.max_speed_hz = 500000

# # # BMP280_REG_CONTROL = 0xF4
# # # BMP280_REG_TEMP = 0xFA
# # # BMP280_REG_PRESS = 0xF7
# # # BMP280_REG_CONFIG = 0xF5
# # # BMP280_REG_ID = 0xD0
# # # BMP280_ID_VALUE = 0x58

# # # def spi_write_register(register, value):
# # #     spi.xfer2([register & 0x7F, value])

# # # def spi_read_register(register):
# # #     return spi.xfer2([register | 0x80, 0x00])[1]

# # # def spi_read_bytes(register, length):
# # #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # # def init_bmp280():
# # #     chip_id = spi_read_register(BMP280_REG_ID)
# # #     if chip_id != BMP280_ID_VALUE:
# # #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# # #         return None

# # #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# # #     spi_write_register(BMP280_REG_CONFIG, 0xA0)
# # #     print("[✅] BMP280 Initialized Successfully")
# # #     return read_calibration_data()

# # # def read_calibration_data():
# # #     calib = spi_read_bytes(0x88, 24)
# # #     dig_T1 = (calib[1] << 8) | calib[0]
# # #     dig_T2 = (calib[3] << 8) | calib[2]
# # #     dig_T3 = (calib[5] << 8) | calib[4]
# # #     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# # # def read_raw_temperature():
# # #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # def compensate_temperature(raw_temp, calib):
# # #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# # #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# # #     t_fine = var1 + var2
# # #     return t_fine / 5120.0  # Celsius

# # # # === STEP 3: Main Logic ===
# # # TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# # # calibration_data = init_bmp280()

# # # if calibration_data:
# # #     try:
# # #         stepper_running = False
# # #         last_action = None

# # #         while True:
# # #             # Read temperature
# # #             raw_temp = read_raw_temperature()
# # #             temp_c = compensate_temperature(raw_temp, calibration_data)
# # #             print(f"[🌡] Temperature: {temp_c:.2f} °C")

# # #             # Control the stepper motor based on temperature
# # #             if temp_c > TEMP_THRESHOLD and not stepper_running:
# # #                 print("[⚙] Temperature is above threshold. Starting stepper motor...")
# # #                 stepper_running = True
# # #                 last_action = "start"
# # #                 rotate_stepper_continuous(delay=0.002)  # Start continuous rotation
# # #             elif temp_c <= TEMP_THRESHOLD and stepper_running:
# # #                 print("[⚙] Temperature is back to normal. Stopping stepper motor...")
# # #                 stepper_running = False
# # #                 last_action = "stop"
# # #                 stop_stepper()

# # #             time.sleep(1)  # Check temperature every second

# # #     except KeyboardInterrupt:
# # #         print("\n[🛑] Exiting program.")
# # #         stepper_running = False  # Ensure the motor stops
# # #         stop_stepper()
# # #         spi.close()






# # # # WORKING THINKSPEAK  CODE :

# # # import time
# # # import spidev
# # # import wiringpi
# # # import paho.mqtt.client as mqtt

# # # # === STEP 1: Initialize Stepper Motor ===
# # # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Setup WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper motor pins as output
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # def rotate_stepper_continuous(delay=0.002):
# # #     """Rotate the stepper motor continuously."""
# # #     while stepper_running:  # Keep rotating as long as stepper_running is True
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # def stop_stepper():
# # #     """Turn off the stepper motor (set all pins to LOW)."""
# # #     for pin in STEPPER_PINS:
# # #         wiringpi.digitalWrite(pin, 0)

# # # # === STEP 2: Initialize BMP280 Sensor ===
# # # SPI_BUS = 1
# # # SPI_DEVICE = 0
# # # spi = spidev.SpiDev()
# # # spi.open(SPI_BUS, SPI_DEVICE)
# # # spi.max_speed_hz = 500000

# # # BMP280_REG_CONTROL = 0xF4
# # # BMP280_REG_TEMP = 0xFA
# # # BMP280_REG_PRESS = 0xF7
# # # BMP280_REG_CONFIG = 0xF5
# # # BMP280_REG_ID = 0xD0
# # # BMP280_ID_VALUE = 0x58

# # # def spi_write_register(register, value):
# # #     spi.xfer2([register & 0x7F, value])

# # # def spi_read_register(register):
# # #     return spi.xfer2([register | 0x80, 0x00])[1]

# # # def spi_read_bytes(register, length):
# # #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # # def init_bmp280():
# # #     chip_id = spi_read_register(BMP280_REG_ID)
# # #     if chip_id != BMP280_ID_VALUE:
# # #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# # #         return None

# # #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# # #     spi_write_register(BMP280_REG_CONFIG, 0xA0)
# # #     print("[✅] BMP280 Initialized Successfully")
# # #     return read_calibration_data()

# # # def read_calibration_data():
# # #     calib = spi_read_bytes(0x88, 24)
# # #     dig_T1 = (calib[1] << 8) | calib[0]
# # #     dig_T2 = (calib[3] << 8) | calib[2]
# # #     dig_T3 = (calib[5] << 8) | calib[4]
# # #     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# # # def read_raw_temperature():
# # #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # def compensate_temperature(raw_temp, calib):
# # #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# # #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# # #     t_fine = var1 + var2
# # #     return t_fine / 5120.0  # Celsius

# # # # === STEP 3: MQTT Setup ===
# # # # ThingSpeak MQTT Configuration
# # # BROKER = "mqtt.thingspeak.com"
# # # PORT = 1883
# # # USERNAME = "EQwsOxw1FA08CxMOKyksNSw"  # Provided by ThingSpeak
# # # PASSWORD = "Tf/XyCitfEV9l+/WLX708Vue"  # Provided by ThingSpeak
# # # CLIENT_ID = "EQwsOxw1FA08CxMOKyksNSw"  # Provided by ThingSpeak
# # # TOPIC = "channels/2792379/publish/fields/field1"  # Updated with Waardes channel (2792379)

# # # # MQTT Client Initialization (updated for compatibility)
# # # mqtt_client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311, transport="tcp")
# # # mqtt_client.username_pw_set(USERNAME, PASSWORD)

# # # # Connect to the MQTT Broker
# # # mqtt_client.connect(BROKER, PORT, 60)
# # # print("[✅] Connected to ThingSpeak MQTT Broker")

# # # # === STEP 4: Main Logic ===
# # # TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# # # calibration_data = init_bmp280()

# # # if calibration_data:
# # #     try:
# # #         stepper_running = False
# # #         last_action = None

# # #         while True:
# # #             # Read temperature
# # #             raw_temp = read_raw_temperature()
# # #             temp_c = compensate_temperature(raw_temp, calibration_data)
# # #             print(f"[🌡] Temperature: {temp_c:.2f} °C")

# # #             # Publish temperature data to ThingSpeak
# # #             mqtt_client.publish(TOPIC, f"field1={temp_c:.2f}")
# # #             print("[📡] Temperature data sent to ThingSpeak (Channel: Waardes 2792379)")

# # #             # Control the stepper motor based on temperature
# # #             if temp_c > TEMP_THRESHOLD and not stepper_running:
# # #                 print("[⚙] Temperature is above threshold. Starting stepper motor...")
# # #                 stepper_running = True
# # #                 last_action = "start"
# # #                 rotate_stepper_continuous(delay=0.002)  # Start continuous rotation
# # #             elif temp_c <= TEMP_THRESHOLD and stepper_running:
# # #                 print("[⚙] Temperature is back to normal. Stopping stepper motor...")
# # #                 stepper_running = False
# # #                 last_action = "stop"
# # #                 stop_stepper()

# # #             time.sleep(1)  # Check temperature every second

# # #     except KeyboardInterrupt:
# # #         print("\n[🛑] Exiting program.")
# # #         stepper_running = False  # Ensure the motor stops
# # #         stop_stepper()
# # #         spi.close()






# # # #working code hopelijke:



# # import time
# # import spidev
# # import wiringpi
# # import paho.mqtt.client as mqtt

# # # === STEP 1: Initialize Stepper Motor ===
# # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # HALF_STEP_SEQUENCE = [
# #     [1, 0, 0, 0],
# #     [1, 1, 0, 0],
# #     [0, 1, 0, 0],
# #     [0, 1, 1, 0],
# #     [0, 0, 1, 0],
# #     [0, 0, 1, 1],
# #     [0, 0, 0, 1],
# #     [1, 0, 0, 1]
# # ]

# # # Setup WiringPi
# # wiringpi.wiringPiSetup()

# # # Set all stepper motor pins as output
# # for pin in STEPPER_PINS:
# #     wiringpi.pinMode(pin, 1)

# # def rotate_stepper_continuous(delay=0.002):
# #     """Rotate the stepper motor continuously."""
# #     while stepper_running:  # Keep rotating as long as stepper_running is True
# #         for step_values in HALF_STEP_SEQUENCE:
# #             for pin, value in zip(STEPPER_PINS, step_values):
# #                 wiringpi.digitalWrite(pin, value)
# #             time.sleep(delay)

# # def stop_stepper():
# #     """Turn off the stepper motor (set all pins to LOW)."""
# #     for pin in STEPPER_PINS:
# #         wiringpi.digitalWrite(pin, 0)

# # # === STEP 2: Initialize BMP280 Sensor ===
# # SPI_BUS = 1
# # SPI_DEVICE = 0
# # spi = spidev.SpiDev()
# # spi.open(SPI_BUS, SPI_DEVICE)
# # spi.max_speed_hz = 500000

# # BMP280_REG_CONTROL = 0xF4
# # BMP280_REG_TEMP = 0xFA
# # BMP280_REG_PRESS = 0xF7
# # BMP280_REG_CONFIG = 0xF5
# # BMP280_REG_ID = 0xD0
# # BMP280_ID_VALUE = 0x58

# # def spi_write_register(register, value):
# #     spi.xfer2([register & 0x7F, value])

# # def spi_read_register(register):
# #     return spi.xfer2([register | 0x80, 0x00])[1]

# # def spi_read_bytes(register, length):
# #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # def init_bmp280():
# #     chip_id = spi_read_register(BMP280_REG_ID)
# #     if chip_id != BMP280_ID_VALUE:
# #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# #         return None

# #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# #     spi_write_register(BMP280_REG_CONFIG, 0xA0)
# #     print("[✅] BMP280 Initialized Successfully")
# #     return read_calibration_data()

# # def read_calibration_data():
# #     calib = spi_read_bytes(0x88, 24)
# #     dig_T1 = (calib[1] << 8) | calib[0]
# #     dig_T2 = (calib[3] << 8) | calib[2]
# #     dig_T3 = (calib[5] << 8) | calib[4]
# #     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# # def read_raw_temperature():
# #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # def compensate_temperature(raw_temp, calib):
# #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# #     t_fine = var1 + var2
# #     return t_fine / 5120.0  # Celsius

# # # === STEP 3: MQTT Setup ===
# # # ThingSpeak MQTT Configuration
# # BROKER = "mqtt.thingspeak.com"
# # PORT = 1883
# # USERNAME = "EQwsOxw1FA08CxMOKyksNSw"  # Your Write API Key
# # PASSWORD = "Fs3XrEIGK3hkSUCvyhEGwEhg"  # Authentication for MQTT
# # CLIENT_ID = "EQwsOxw1FA08CxMOKyksNSw"  # Provided by ThingSpeak
# # TOPIC = "channels/2792379/publish/"  # Adjusted topic for Channel ID: 2792379

# # # MQTT Client Initialization (updated for compatibility)
# # mqtt_client = mqtt.Client(client_id=CLIENT_ID, protocol=mqtt.MQTTv311, transport="tcp")
# # mqtt_client.username_pw_set(USERNAME, PASSWORD)

# # # Connect to the MQTT Broker
# # mqtt_client.connect(BROKER, PORT, 60)
# # print("[✅] Connected to ThingSpeak MQTT Broker")

# # # === STEP 4: Main Logic ===
# # TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# # calibration_data = init_bmp280()

# # if calibration_data:
# #     try:
# #         stepper_running = False
# #         last_action = None

# #         while True:
# #             # Read temperature
# #             raw_temp = read_raw_temperature()
# #             temp_c = compensate_temperature(raw_temp, calibration_data)
# #             print(f"[🌡] Temperature: {temp_c:.2f} °C")

# #             # Publish temperature data to ThingSpeak
# #             payload = f"field4={temp_c:.2f}"
# #             mqtt_client.publish(TOPIC, payload)
# #             print(f"[📡] Sent to ThingSpeak: {payload}")

# #             # Control the stepper motor based on temperature
# #             if temp_c > TEMP_THRESHOLD and not stepper_running:
# #                 print("[⚙] Temperature is above threshold. Starting stepper motor...")
# #                 stepper_running = True
# #                 last_action = "start"
# #                 rotate_stepper_continuous(delay=0.002)  # Start continuous rotation
# #             elif temp_c <= TEMP_THRESHOLD and stepper_running:
# #                 print("[⚙] Temperature is back to normal. Stopping stepper motor...")
# #                 stepper_running = False
# #                 last_action = "stop"
# #                 stop_stepper()

# #             time.sleep(15)  # Send data to ThingSpeak every 15 seconds

# #     except KeyboardInterrupt:
# #         print("\n[🛑] Exiting program.")
# #         stepper_running = False  # Ensure the motor stops
# #         stop_stepper()
# #         spi.close()

# # # import time
# # # import spidev
# # # import wiringpi
# # # import paho.mqtt.client as mqtt

# # # # === STEP 1: Initialize Stepper Motor ===
# # # STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
# # # HALF_STEP_SEQUENCE = [
# # #     [1, 0, 0, 0],
# # #     [1, 1, 0, 0],
# # #     [0, 1, 0, 0],
# # #     [0, 1, 1, 0],
# # #     [0, 0, 1, 0],
# # #     [0, 0, 1, 1],
# # #     [0, 0, 0, 1],
# # #     [1, 0, 0, 1]
# # # ]

# # # # Setup WiringPi
# # # wiringpi.wiringPiSetup()

# # # # Set all stepper motor pins as output
# # # for pin in STEPPER_PINS:
# # #     wiringpi.pinMode(pin, 1)

# # # def rotate_stepper_continuous(delay=0.002):
# # #     """Rotate the stepper motor continuously."""
# # #     while stepper_running:  # Keep rotating as long as stepper_running is True
# # #         for step_values in HALF_STEP_SEQUENCE:
# # #             for pin, value in zip(STEPPER_PINS, step_values):
# # #                 wiringpi.digitalWrite(pin, value)
# # #             time.sleep(delay)

# # # def stop_stepper():
# # #     """Turn off the stepper motor (set all pins to LOW)."""
# # #     for pin in STEPPER_PINS:
# # #         wiringpi.digitalWrite(pin, 0)

# # # # === STEP 2: Initialize BMP280 Sensor ===
# # # SPI_BUS = 1
# # # SPI_DEVICE = 0
# # # spi = spidev.SpiDev()
# # # spi.open(SPI_BUS, SPI_DEVICE)
# # # spi.max_speed_hz = 500000

# # # BMP280_REG_CONTROL = 0xF4
# # # BMP280_REG_TEMP = 0xFA
# # # BMP280_REG_PRESS = 0xF7
# # # BMP280_REG_CONFIG = 0xF5
# # # BMP280_REG_ID = 0xD0
# # # BMP280_ID_VALUE = 0x58

# # # def spi_write_register(register, value):
# # #     spi.xfer2([register & 0x7F, value])

# # # def spi_read_register(register):
# # #     return spi.xfer2([register | 0x80, 0x00])[1]

# # # def spi_read_bytes(register, length):
# # #     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # # def init_bmp280():
# # #     chip_id = spi_read_register(BMP280_REG_ID)
# # #     if chip_id != BMP280_ID_VALUE:
# # #         print(f"[⚠] Error: Device ID mismatch! Expected 0x58, got {chip_id:#02x}")
# # #         return None

# # #     spi_write_register(BMP280_REG_CONTROL, 0x27)
# # #     spi_write_register(BMP280_REG_CONFIG, 0xA0)
# # #     print("[✅] BMP280 Initialized Successfully")
# # #     return read_calibration_data()

# # # def read_calibration_data():
# # #     calib = spi_read_bytes(0x88, 24)
# # #     dig_T1 = (calib[1] << 8) | calib[0]
# # #     dig_T2 = (calib[3] << 8) | calib[2]
# # #     dig_T3 = (calib[5] << 8) | calib[4]
# # #     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# # # def read_raw_temperature():
# # #     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
# # #     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # # def compensate_temperature(raw_temp, calib):
# # #     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
# # #     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
# # #     t_fine = var1 + var2
# # #     return t_fine / 5120.0  # Celsius

# # # # === STEP 3: MQTT Setup ===
# # # # ThingSpeak MQTT Configuration
# # # BROKER = "mqtt.thingspeak.com"
# # # PORT = 1883
# # # USERNAME = "EQwsOxw1FA08CxMOKyksNSw"  # Your Write API Key
# # # PASSWORD = "oeZAeVCH/tPPfhFkymhE/Lxj"  # Authentication for MQTT
# # # CLIENT_ID = "EQwsOxw1FA08CxMOKyksNSw"  # Provided by ThingSpeak
# # # TOPIC = "channels/2792379/publish"  # Adjusted topic for Channel ID: 2792379

# # # # MQTT Client Initialization (updated for compatibility)
# # # mqtt_client = mqtt.Client(client_id=CLIENT_ID)  # Updated to use the latest API
# # # mqtt_client.username_pw_set(USERNAME, PASSWORD)

# # # # Connect to the MQTT Broker
# # # mqtt_client.connect(BROKER, PORT, 60)
# # # mqtt_client.loop_start()  # Start the MQTT client loop to keep the connection alive
# # # print("[✅] Connected to ThingSpeak MQTT Broker")

# # # # === STEP 4: Main Logic ===
# # # TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# # # calibration_data = init_bmp280()

# # # if calibration_data:
# # #     try:
# # #         stepper_running = False
# # #         last_action = None

# # #         while True:
# # #             # Read temperature
# # #             raw_temp = read_raw_temperature()
# # #             temp_c = compensate_temperature(raw_temp, calibration_data)
# # #             print(f"[🌡] Temperature: {temp_c:.2f} °C")

# # #             # Publish temperature data to ThingSpeak
# # #             payload = f"field 4 ={temp_c:.2f}"
# # #             mqtt_client.publish(TOPIC, payload)
# # #             print(f"[📡] Sent to ThingSpeak: {payload}")

# # #             # Control the stepper motor based on temperature
# # #             if temp_c > TEMP_THRESHOLD and not stepper_running:
# # #                 print("[⚙] Temperature is above threshold. Starting stepper motor...")
# # #                 stepper_running = True
# # #                 last_action = "start"
# # #                 rotate_stepper_continuous(delay=0.002)  # Start continuous rotation
# # #             elif temp_c <= TEMP_THRESHOLD and stepper_running:
# # #                 print("[⚙] Temperature is back to normal. Stopping stepper motor...")
# # #                 stepper_running = False
# # #                 last_action = "stop"
# # #                 stop_stepper()

# # #             time.sleep(15)  # Send data to ThingSpeak every 15 seconds

# # #     except KeyboardInterrupt:
# # #         print("\n[🛑] Exiting program.")
# # #         stepper_running = False  # Ensure the motor stops
# # #         stop_stepper()
# # #         spi.close()

#FUCKING PERFECT CODE BELOW SENDING DATA TO THINKSPEAK !!!

# import time
# import spidev
# import wiringpi
# import requests  # ✅ Added for HTTP requests

# # === STEP 1: Initialize Stepper Motor ===
# STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
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

# # Setup WiringPi
# wiringpi.wiringPiSetup()

# # Set all stepper motor pins as output
# for pin in STEPPER_PINS:
#     wiringpi.pinMode(pin, 1)

# def rotate_stepper_continuous(delay=0.002):
#     """Rotate the stepper motor continuously."""
#     while stepper_running:
#         for step_values in HALF_STEP_SEQUENCE:
#             for pin, value in zip(STEPPER_PINS, step_values):
#                 wiringpi.digitalWrite(pin, value)
#             time.sleep(delay)

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

#     spi_write_register(BMP280_REG_CONTROL, 0x27)  # Normal mode, 1x oversampling
#     print("[✅] BMP280 Initialized Successfully")
#     return read_calibration_data()

# def read_calibration_data():
#     calib = spi_read_bytes(0x88, 6)  # Only temperature calibration registers
#     dig_T1 = (calib[1] << 8) | calib[0]
#     dig_T2 = (calib[3] << 8) | calib[2]
#     dig_T3 = (calib[5] << 8) | calib[4]
#     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# def read_raw_temperature():
#     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
#     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# def compensate_temperature(raw_temp, calib):
#     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
#     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
#     t_fine = var1 + var2
#     return t_fine / 5120.0  # Celsius

# # === STEP 3: Main Logic ===
# TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# calibration_data = init_bmp280()

# if calibration_data:
#     try:
#         stepper_running = False

#         while True:
#             raw_temp = read_raw_temperature()
#             temp_c = compensate_temperature(raw_temp, calibration_data)
#             print(f"[🌡] Temperature: {temp_c:.2f} °C")

#             # ✅ Send temperature to ThingSpeak using full URL (field4)
#             THINGSPEAK_URL = f"https://api.thingspeak.com/update?api_key=LKPW521GXHAU6G31&field4={temp_c:.2f}"
#             response = requests.get(THINGSPEAK_URL)
#             print(f"[📡] Sent to ThingSpeak (field4): {temp_c:.2f} °C | Response: {response.status_code}")

#             # Control the stepper motor based on temperature
#             if temp_c > TEMP_THRESHOLD and not stepper_running:
#                 print("[⚙] Temperature above threshold. Starting stepper motor...")
#                 stepper_running = True
#                 rotate_stepper_continuous(delay=0.002)
#             elif temp_c <= TEMP_THRESHOLD and stepper_running:
#                 print("[⚙] Temperature normal. Stopping stepper motor...")
#                 stepper_running = False
#                 stop_stepper()

#             time.sleep(15)

#     except KeyboardInterrupt:
#         print("\n[🛑] Exiting program.")
#         stepper_running = False
#         stop_stepper()
#         spi.close()


#AMAZING CODE: 

# import time
# import spidev
# import wiringpi
# import requests  # ✅ Added for HTTP requests

# # === STEP 1: Initialize Stepper Motor ===
# STEPPER_PINS = [3, 4, 5, 6]  # GPIO pins for ULN2003 IN1, IN2, IN3, IN4
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

# # Setup WiringPi
# wiringpi.wiringPiSetup()

# # Set all stepper motor pins as output
# for pin in STEPPER_PINS:
#     wiringpi.pinMode(pin, 1)

# def rotate_stepper_continuous(delay=0.002):
#     """Rotate the stepper motor continuously."""
#     while stepper_running:
#         for step_values in HALF_STEP_SEQUENCE:
#             for pin, value in zip(STEPPER_PINS, step_values):
#                 wiringpi.digitalWrite(pin, value)
#             time.sleep(delay)

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

#     spi_write_register(BMP280_REG_CONTROL, 0x27)  # Normal mode, 1x oversampling
#     print("[✅] BMP280 Initialized Successfully")
#     return read_calibration_data()

# def read_calibration_data():
#     calib = spi_read_bytes(0x88, 6)  # Only temperature calibration registers
#     dig_T1 = (calib[1] << 8) | calib[0]
#     dig_T2 = (calib[3] << 8) | calib[2]
#     dig_T3 = (calib[5] << 8) | calib[4]
#     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# def read_raw_temperature():
#     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
#     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# def compensate_temperature(raw_temp, calib):
#     var1 = (((raw_temp / 16384.0) - (calib["T1"] / 1024.0)) * calib["T2"])
#     var2 = (((raw_temp / 131072.0) - (calib["T1"] / 8192.0)) ** 2) * calib["T3"]
#     t_fine = var1 + var2
#     return t_fine / 5120.0  # Celsius

# # === STEP 3: Main Logic ===
# TEMP_THRESHOLD = 25.0  # Temperature threshold in Celsius
# desired_temperature = TEMP_THRESHOLD
# calibration_data = init_bmp280()

# BUTTON_INCREASE = 7
# BUTTON_DECREASE = 8
# wiringpi.pinMode(BUTTON_INCREASE, 0)
# wiringpi.pinMode(BUTTON_DECREASE, 0)

# if calibration_data:
#     try:
#         stepper_running = False

#         while True:
#             if wiringpi.digitalRead(BUTTON_INCREASE) == 1:
#                 desired_temperature += 1
#             if wiringpi.digitalRead(BUTTON_DECREASE) == 1:
#                 desired_temperature -= 1

#             raw_temp = read_raw_temperature()
#             temp_c = compensate_temperature(raw_temp, calibration_data)
#             print(f"[🌡] Temperature: {temp_c:.2f} °C | Desired: {desired_temperature:.2f} °C")

#             THINGSPEAK_URL_SENSOR = f"https://api.thingspeak.com/update?api_key=LKPW521GXHAU6G31&field4={temp_c:.2f}"
#             THINGSPEAK_URL_DESIRED = f"https://api.thingspeak.com/update?api_key=2HBKEUX8ENIXKAGX&field4={desired_temperature:.2f}"
#             requests.get(THINGSPEAK_URL_SENSOR)
#             requests.get(THINGSPEAK_URL_DESIRED)

#             if temp_c > desired_temperature and not stepper_running:
#                 print("[⚙] Temperature above threshold. Starting stepper motor...")
#                 stepper_running = True
#                 rotate_stepper_continuous(delay=0.002)
#             elif temp_c <= desired_temperature and stepper_running:
#                 print("[⚙] Temperature normal. Stopping stepper motor...")
#                 stepper_running = False
#                 stop_stepper()

#             time.sleep(15)

#     except KeyboardInterrupt:
#         print("\n[🛑] Exiting program.")
#         stepper_running = False
#         stop_stepper()
#         spi.close()

# # SECOND AMAZING CODE :

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


# # # import time
# # # from bmp280 import BMP280
# # # from smbus2 import SMBus
# # # import paho.mqtt.client as mqtt

# # # # Create an I2C bus object
# # # bus = SMBus(1)  # Use the correct I2C bus number, typically 1 for Raspberry Pi
# # # address = 0x77  # BMP280 I2C address

# # # # Setup BMP280
# # # bmp280 = BMP280(i2c_addr=address, i2c_dev=bus)
# # # interval = 15  # Sample period in seconds

# # # # MQTT setup with your credentials
# # # MQTT_BROKER = "mqtt.thingspeak.com"
# # # MQTT_PORT = 1883
# # # MQTT_CLIENT_ID = "EQwsOxw1FA08CxMOKyksNSw"
# # # MQTT_USERNAME = "EQwsOxw1FA08CxMOKyksNSw"
# # # MQTT_PASSWORD = "FnINi8Mhxc5Vqeyztv8avu/J"
# # # MQTT_TOPIC = "channels/2792379/publish"  # Use your Channel ID

# # # # Initialize MQTT client
# # # client = mqtt.Client(client_id=MQTT_CLIENT_ID)
# # # client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
# # # client.connect(MQTT_BROKER, MQTT_PORT, 60)

# # # while True:
# # #     # Measure data
# # #     bmp280_temperature = bmp280.get_temperature()
# # #     bmp280_pressure = bmp280.get_pressure()
# # #     print(f"Temperature: {bmp280_temperature:4.1f}, Pressure: {bmp280_pressure:4.1f}")
    
# # #     # Create the MQTT payload
# # #     MQTT_DATA = f"field1={bmp280_temperature}&field2={bmp280_pressure}&status=MQTTPUBLISH"
# # #     print(MQTT_DATA)
    
# # #     try:
# # #         # Publish data to ThingSpeak
# # #         client.publish(topic=MQTT_TOPIC, payload=MQTT_DATA, qos=0, retain=False)
# # #         time.sleep(interval)
# # #     except OSError:
# # #         # Reconnect if there's an error
# # #         client.reconnect()




# import time
# import spidev
# import wiringpi
# import paho.mqtt.client as mqtt

# # ---------------------
# # STEP 1: STEPPER MOTOR
# # ---------------------
# STEPPER_PINS = [3, 4, 5, 6]  # Adjust these for your wiring
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
#     """Motor spins until 'stepper_running' becomes False (blocking)."""
#     while stepper_running:
#         for step_values in HALF_STEP_SEQUENCE:
#             for pin, val in zip(STEPPER_PINS, step_values):
#                 wiringpi.digitalWrite(pin, val)
#             time.sleep(delay)

# def stop_stepper():
#     """Turn all stepper motor pins LOW."""
#     for pin in STEPPER_PINS:
#         wiringpi.digitalWrite(pin, 0)


# # ------------------
# # STEP 2: BMP280 SPI
# # ------------------
# SPI_BUS = 1            # If your system only has /dev/spidev0.0, use 0 here
# SPI_DEVICE = 0
# spi = spidev.SpiDev()
# spi.open(SPI_BUS, SPI_DEVICE)
# spi.max_speed_hz = 500000

# BMP280_REG_CONTROL = 0xF4
# BMP280_REG_CONFIG  = 0xF5
# BMP280_REG_TEMP    = 0xFA
# BMP280_REG_ID      = 0xD0
# BMP280_ID_VALUE    = 0x58

# def spi_write_register(reg, val):
#     spi.xfer2([reg & 0x7F, val])

# def spi_read_register(reg):
#     return spi.xfer2([reg | 0x80, 0x00])[1]

# def spi_read_bytes(reg, length):
#     return spi.xfer2([reg | 0x80] + [0x00]*length)[1:]

# def init_bmp280():
#     chip_id = spi_read_register(BMP280_REG_ID)
#     if chip_id != BMP280_ID_VALUE:
#         print(f"[⚠] BMP280 ID mismatch! Got 0x{chip_id:02X}, expected 0x58.")
#         return None
    
#     # Normal mode, oversampling=1 => 0x27, config=0xA0
#     spi_write_register(BMP280_REG_CONTROL, 0x27)
#     spi_write_register(BMP280_REG_CONFIG,  0xA0)
#     print("[✅] BMP280 initialized successfully")
#     return read_calibration_data()

# def read_calibration_data():
#     raw_calib = spi_read_bytes(0x88, 24)
#     dig_T1 = (raw_calib[1] << 8) | raw_calib[0]
#     dig_T2 = (raw_calib[3] << 8) | raw_calib[2]
#     dig_T3 = (raw_calib[5] << 8) | raw_calib[4]
#     return {"T1": dig_T1, "T2": dig_T2, "T3": dig_T3}

# def read_raw_temperature():
#     raw = spi_read_bytes(BMP280_REG_TEMP, 3)
#     # 20-bit raw temperature
#     return (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)

# def compensate_temperature(raw_temp, c):
#     var1 = ((raw_temp / 16384.0) - (c["T1"] / 1024.0)) * c["T2"]
#     var2 = (((raw_temp / 131072.0) - (c["T1"] / 8192.0)) ** 2) * c["T3"]
#     t_fine = var1 + var2
#     return t_fine / 5120.0  # Celsius


# # ---------------------------------
# # STEP 3: MQTT SETUP (Key in Payload)
# # ---------------------------------
# BROKER         = "mqtt.thingspeak.com"
# PORT           = 1883
# CHANNEL_ID     = "2832083"            # <-- Your new channel ID
# WRITE_API_KEY  = "72YMCSMJ9MGAHGF9"   # <-- Your new channel's Write API Key

# # Because we put the API key in the payload, the topic is just "channels/ID/publish"
# TOPIC = f"channels/{CHANNEL_ID}/publish"

# mqtt_client = mqtt.Client()

# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         print("[✅] Connected to ThingSpeak MQTT Broker")
#     else:
#         print(f"[❌] Connection failed with code {rc}")

# def on_publish(client, userdata, mid):
#     print(f"[MQTT] Publish successful (mid={mid})")

# def on_disconnect(client, userdata, rc):
#     print(f"[MQTT] Disconnected with rc={rc}")

# mqtt_client.on_connect = on_connect
# mqtt_client.on_publish = on_publish
# mqtt_client.on_disconnect = on_disconnect

# mqtt_client.connect(BROKER, PORT, 60)
# mqtt_client.loop_start()

# # --------------------------
# # STEP 4: MAIN LOOP
# # --------------------------
# TEMP_THRESHOLD = 25.0
# calibration_data = init_bmp280()
# if not calibration_data:
#     print("[❌] BMP280 not found. Exiting.")
#     spi.close()
#     exit(1)

# stepper_running = False

# try:
#     while True:
#         # 1) Read BMP280 temperature
#         raw_temp = read_raw_temperature()
#         temp_c   = compensate_temperature(raw_temp, calibration_data)
#         print(f"[🌡] Temperature: {temp_c:.2f} °C")

#         # 2) Publish temperature to ThingSpeak in field1.
#         #    "api_key=..." because we are not using key in the topic.
#         payload = f"api_key={WRITE_API_KEY}&field1={temp_c:.2f}"
#         result = mqtt_client.publish(TOPIC, payload)
#         if result.rc == 0:
#             print(f"[📡] Sent to ThingSpeak: {payload}")
#         else:
#             print(f"[🛑] Publish failed. Error code: {result.rc}")

#         # 3) Stepper motor control above threshold
#         if temp_c > TEMP_THRESHOLD and not stepper_running:
#             print("[⚙] Temp above threshold -> starting stepper")
#             stepper_running = True
#             rotate_stepper_continuous(0.002)  # blocks until stepper_running = False
#         elif temp_c <= TEMP_THRESHOLD and stepper_running:
#             print("[⚙] Temp normal -> stopping stepper")
#             stepper_running = False
#             stop_stepper()

#         # 4) Delay to avoid rate-limit disconnection
#         time.sleep(60)  # one minute

# except KeyboardInterrupt:
#     print("\n[🛑] Exiting by user request.")

# finally:
#     stepper_running = False
#     stop_stepper()
#     spi.close()
#     mqtt_client.loop_stop()
#     mqtt_client.disconnect()
#     print("[✅] Done. Goodbye!")







# import time
# import spidev
# import wiringpi
# import paho.mqtt.client as mqtt
# import threading
# import socket

# # === MQTT Credentials ===
# MQTT_CLIENT_ID = "FCYMDSUqPS8BGAM2KgkiAAk"
# MQTT_USERNAME = "FCYMDSUqPS8BGAM2KgkiAAk"
# MQTT_PASSWORD = "JT+CQSNEmuT/BCU4v6fSB6RW"
# MQTT_BROKER = "mqtt3.thingspeak.com"
# MQTT_PORT = 1883
# THINGSPEAK_TOPIC = "channels/2832083/publish"

# # === Initialize MQTT Client ===
# def on_connect(client, userdata, flags, rc):
#     if rc == 0:
#         print("[✅] Connected to ThingSpeak MQTT Broker")
#     else:
#         print(f"[❌] Connection failed with error code {rc}")

# client = mqtt.Client(client_id=MQTT_CLIENT_ID)
# client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
# client.on_connect = on_connect
# client.connect(MQTT_BROKER, MQTT_PORT, 60)
# client.loop_start()

# # === Initialize Stepper Motor ===
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
#     global stepper_running
#     while stepper_running:
#         for step_values in HALF_STEP_SEQUENCE:
#             for pin, value in zip(STEPPER_PINS, step_values):
#                 wiringpi.digitalWrite(pin, value)
#             time.sleep(delay)

# def stop_stepper():
#     for pin in STEPPER_PINS:
#         wiringpi.digitalWrite(pin, 0)

# # === Initialize BMP280 Sensor ===
# SPI_BUS = 1
# SPI_DEVICE = 0
# spi = spidev.SpiDev()
# spi.open(SPI_BUS, SPI_DEVICE)
# spi.max_speed_hz = 500000

# BMP280_REG_TEMP = 0xFA
# BMP280_REG_CALIB = 0x88  # Calibration data start register

# def spi_read_bytes(register, length):
#     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# # === Read Calibration Data ===
# def read_calibration_data():
#     calib = spi_read_bytes(BMP280_REG_CALIB, 24)
#     dig_T1 = (calib[1] << 8) | calib[0]
#     dig_T2 = (calib[3] << 8) | calib[2]
#     dig_T3 = (calib[5] << 8) | calib[4]
#     return dig_T1, dig_T2, dig_T3

# dig_T1, dig_T2, dig_T3 = read_calibration_data()

# # === Read Raw Temperature ===
# def read_raw_temperature():
#     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
#     return (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)

# # === Corrected Compensation Formula ===
# def compensate_temperature(raw_temp):
#     global dig_T1, dig_T2, dig_T3
#     var1 = (((raw_temp / 16384.0) - (dig_T1 / 1024.0)) * dig_T2)
#     var2 = ((((raw_temp / 131072.0) - (dig_T1 / 8192.0)) ** 2) * dig_T3)
#     t_fine = var1 + var2
#     temperature = t_fine / 5120.0
#     return temperature

# # === Main Loop ===
# TEMP_THRESHOLD = 25.0
# stepper_running = False

# try:
#     while True:
#         raw_temp = read_raw_temperature()
#         temp_c = compensate_temperature(raw_temp)
#         print(f"[🌡] Temperature: {temp_c:.2f} °C")

#         # Publish data to ThingSpeak via MQTT
#         payload = f"field4={temp_c:.2f}"
#         result = client.publish(THINGSPEAK_TOPIC, payload)
#         if result.rc == mqtt.MQTT_ERR_SUCCESS:
#             print("[📡] Sent to ThingSpeak via MQTT")
#         else:
#             print("[❌] MQTT Publish Failed")

#         # Stepper Motor Control
#         if temp_c > TEMP_THRESHOLD and not stepper_running:
#             print("[⚙] Temperature high! Starting stepper motor...")
#             stepper_running = True
#             threading.Thread(target=rotate_stepper_continuous, daemon=True).start()
#         elif temp_c <= TEMP_THRESHOLD and stepper_running:
#             print("[⚙] Temperature normal. Stopping stepper motor...")
#             stepper_running = False
#             stop_stepper()

#         time.sleep(15)

# except KeyboardInterrupt:
#     print("\n[🛑] Exiting...")
#     stepper_running = False
#     stop_stepper()
#     spi.close()
#     client.loop_stop()






# ZUBAIR CODE: 


# import wiringpi
# import time
# import requests
# import subprocess
# from ch7_ClassLCD import LCD

# def ActivateLCD():
#     wiringpi.digitalWrite(pin_CS_lcd, 0)  # Activate LCD using CS
#     time.sleep(0.000005)

# def DeactivateLCD():
#     wiringpi.digitalWrite(pin_CS_lcd, 1)  # Deactivate LCD using CS
#     time.sleep(0.000005)

# PIN_OUT = {
#     'SCLK': 14,
#     'DIN': 11,
#     'DC': 13,
#     'CS': 15,
#     'RST': 10,
#     'LED': 16
# }

# # Initialize WiringPi
# wiringpi.wiringPiSetup()

# # Set up GPIO
# IN1 = 2
# IN2 = 3
# IN3 = 4
# IN4 = 6
# TRIG = 8
# ECHO = 9
# BUTTON_PIN = 0

# wiringpi.pinMode(TRIG, wiringpi.OUTPUT)
# wiringpi.pinMode(ECHO, wiringpi.INPUT)
# wiringpi.pinMode(IN1, wiringpi.OUTPUT)
# wiringpi.pinMode(IN2, wiringpi.OUTPUT)
# wiringpi.pinMode(IN3, wiringpi.OUTPUT)
# wiringpi.pinMode(IN4, wiringpi.OUTPUT)
# wiringpi.pinMode(BUTTON_PIN, wiringpi.INPUT)

# # Define the URL and UID for the Ubeac server
# url = "http://live9view.hub.ubeac.io/IotZubair"
# uid = "IotZubair"

# # Set up LCD
# pin_CS_lcd = PIN_OUT['CS']
# wiringpi.pinMode(pin_CS_lcd, wiringpi.OUTPUT)
# ActivateLCD()
# lcd = LCD(PIN_OUT)
# lcd.clear()
# lcd.set_backlight(1)
# def setStep(w1, w2, w3, w4):
#     wiringpi.digitalWrite(IN1, w1)
#     wiringpi.digitalWrite(IN2, w2)
#     wiringpi.digitalWrite(IN3, w3)
#     wiringpi.digitalWrite(IN4, w4)

# def rotate_clockwise(steps):
#     for i in range(steps):
#         setStep(1,0,0,0)
#         time.sleep(0.001)
#         setStep(1,1,0,0)
#         time.sleep(0.001)
#         setStep(0,1,0,0)
#         time.sleep(0.001)
#         setStep(0,1,1,0)
#         time.sleep(0.001)
#         setStep(0,0,1,0)
#         time.sleep(0.001)
#         setStep(0,0,1,1)
#         time.sleep(0.001)
#         setStep(0,0,0,1)
#         time.sleep(0.001)
#         setStep(1,0,0,1)
#         time.sleep(0.001)
#     stop_motor()

# def rotate_anticlockwise(steps):
#     for i in range(steps):
#         setStep(1,0,0,1)
#         time.sleep(0.001)
#         setStep(0,0,0,1)
#         time.sleep(0.001)
#         setStep(0,0,1,1)
#         time.sleep(0.001)
#         setStep(0,0,1,0)
#         time.sleep(0.001)
#         setStep(0,1,1,0)
#         time.sleep(0.001)
#         setStep(0,1,0,0)
#         time.sleep(0.001)
#         setStep(1,1,0,0)
#         time.sleep(0.001)
#         setStep(1,0,0,0)
#         time.sleep(0.001)
#     stop_motor()

# def stop_motor():
#     setStep(0, 0, 0, 0)

# def get_distance():
#     wiringpi.digitalWrite(TRIG, wiringpi.HIGH)
#     time.sleep(0.00001)
#     wiringpi.digitalWrite(TRIG, wiringpi.LOW)
#     start_time = time.time()
#     stop_time = time.time()

#     while wiringpi.digitalRead(ECHO) == wiringpi.LOW:
#         start_time = time.time()

#     while wiringpi.digitalRead(ECHO) == wiringpi.HIGH:
#         stop_time = time.time()

#     elapsed_time = stop_time - start_time
#     distance = (elapsed_time * 34300) / 2  # Distance in cm

#     return distance

# def send_data(distance, trap_status):
#     data = {
#         "id": uid,
#         "sensors": [
#             {"id": "Distance", "data": distance},
#             {"id": "Trap Status", "data": trap_status}
#         ]
#     }
#     r = requests.post(url, json=data)
#     if r.status_code == 200:
#         print("Data sent successfully")
#     else:
#         print("Failed to send data")

# # Start MJPG Streamer subprocess
# mjpg_streamer_process = subprocess.Popen([
#     "./mjpg_streamer",
#     "-i", "./input_uvc.so -d /dev/video1",
#     "-o", "./output_http.so -w ./www"
# ])

# previous_distance = get_distance()
# object_detected = False
# door_closing = False

# try:
#     trap_status = ""
#     while True:
#         current_time = time.strftime("%H:%M:%S")
#         distance = get_distance()

#         if wiringpi.digitalRead(BUTTON_PIN) == wiringpi.LOW:
#             rotate_clockwise(700 * 2)
#             trap_status = "Door opened"
#             send_data(distance, trap_status)
#             print("Door opened")
#  elif distance <= 4 and not object_detected:
#             rotate_anticlockwise(700 * 2)
#             trap_status = "Captured"
#             send_data(distance, trap_status)
#             object_detected = True
#             print("Object detected at distance: %.1f cm" % distance)
#         elif distance > 4 and object_detected and not door_closing:
#             stop_motor()
#             trap_status = "Door closed"
#             send_data(distance, trap_status)
#             door_closing = True
#             print("Door closed")
#         elif distance <= 4 and not object_detected and door_closing:
#             rotate_anticlockwise(700 * 2)
#             trap_status = "Captured"
#             send_data(distance, trap_status)
#             object_detected = True
#             door_closing = False
#             print("Object detected at distance: %.1f cm" % distance)
#         elif distance > 4 and object_detected and door_closing:
#             door_closing = False
#             object_detected = False
#             print("Press the button to open the door")

#         ActivateLCD()
#         lcd.clear()
#         lcd.go_to_xy(0, 0)
#         lcd.put_string('Time: ' + current_time)

#         lcd.go_to_xy(0, 1 + 9)
#         lcd.put_string('Trap Status: ' + trap_status)
#         lcd.go_to_xy(0, 2 + 9 + 20)
#         lcd.put_string('Distance: %.1f cm' % distance)
#         lcd.refresh()
#         DeactivateLCD()

#         time.sleep(1)
# except KeyboardInterrupt:
#     # Terminate MJPG Streamer subprocess
#     mjpg_streamer_process.terminate()
#     mjpg_streamer_process.wait()

#     lcd.clear()
#     lcd.refresh()
#     lcd.set_backlight(0)
#     DeactivateLCD()
#     stop_motor()
#     print("\nProgram terminated")




# Sending temperature reading to zubair Ubeac platform

# print("[▶] Script started")

# import wiringpi
# import time
# import requests
# import spidev
# import threading

# # GPIO Setup
# IN1 = 2
# IN2 = 3
# IN3 = 4
# IN4 = 6
# BUTTON_PIN = 0

# wiringpi.wiringPiSetup()
# wiringpi.pinMode(IN1, wiringpi.OUTPUT)
# wiringpi.pinMode(IN2, wiringpi.OUTPUT)
# wiringpi.pinMode(IN3, wiringpi.OUTPUT)
# wiringpi.pinMode(IN4, wiringpi.OUTPUT)
# wiringpi.pinMode(BUTTON_PIN, wiringpi.INPUT)

# # Ubeac Server Details
# url = "http://live9view.hub.ubeac.io/IotZubair"
# uid = "IotZubair"

# # Stepper Motor Control
# def setStep(w1, w2, w3, w4):
#     wiringpi.digitalWrite(IN1, w1)
#     wiringpi.digitalWrite(IN2, w2)
#     wiringpi.digitalWrite(IN3, w3)
#     wiringpi.digitalWrite(IN4, w4)

# def rotate_clockwise(steps):
#     print(f"[↻] Rotating clockwise ({steps} steps)")
#     for _ in range(steps):
#         setStep(1, 0, 0, 0)
#         time.sleep(0.001)
#         setStep(1, 1, 0, 0)
#         time.sleep(0.001)
#         setStep(0, 1, 0, 0)
#         time.sleep(0.001)
#         setStep(0, 1, 1, 0)
#         time.sleep(0.001)
#         setStep(0, 0, 1, 0)
#         time.sleep(0.001)
#         setStep(0, 0, 1, 1)
#         time.sleep(0.001)
#         setStep(0, 0, 0, 1)
#         time.sleep(0.001)
#         setStep(1, 0, 0, 1)
#         time.sleep(0.001)
#     stop_motor()

# def rotate_anticlockwise(steps):
#     print(f"[↺] Rotating anti-clockwise ({steps} steps)")
#     for _ in range(steps):
#         setStep(1, 0, 0, 1)
#         time.sleep(0.001)
#         setStep(0, 0, 0, 1)
#         time.sleep(0.001)
#         setStep(0, 0, 1, 1)
#         time.sleep(0.001)
#         setStep(0, 0, 1, 0)
#         time.sleep(0.001)
#         setStep(0, 1, 1, 0)
#         time.sleep(0.001)
#         setStep(0, 1, 0, 0)
#         time.sleep(0.001)
#         setStep(1, 1, 0, 0)
#         time.sleep(0.001)
#         setStep(1, 0, 0, 0)
#         time.sleep(0.001)
#     stop_motor()

# def stop_motor():
#     setStep(0, 0, 0, 0)
#     print("[⏹] Motor stopped")

# # BMP280 SPI Temperature Sensor (Corrected)
# SPI_BUS = 1
# SPI_DEVICE = 0
# spi = spidev.SpiDev()
# spi.open(SPI_BUS, SPI_DEVICE)
# spi.max_speed_hz = 500000
# BMP280_REG_TEMP = 0xFA

# def spi_read_bytes(register, length):
#     return spi.xfer2([register | 0x80] + [0x00] * length)[1:]

# def read_raw_temperature():
#     raw_data = spi_read_bytes(BMP280_REG_TEMP, 3)
#     raw_temp = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4)
#     return raw_temp

# def compensate_temperature(raw_temp):
#     var1 = (raw_temp / 16384.0 - 25.0) * 1.5  # Apply proper scaling
#     temp_c = round(var1, 2)  
#     if temp_c < -40 or temp_c > 85:  
#         print("[⚠] Temperature out of range, reading ignored!")  
#         return None  
#     return temp_c  

# # Send Data to Ubeac
# def send_data(temp_c, trap_status):
#     if temp_c is None:
#         return  

#     data = {
#         "id": uid,
#         "sensors": [
#             {"id": "Temperature", "data": temp_c},
#             {"id": "Trap Status", "data": trap_status}
#         ]
#     }

#     print(f"[📡] Sending Data: {data}")
#     r = requests.post(url, json=data)

#     if r.status_code == 200:
#         print("[✅] Data sent successfully")
#     else:
#         print(f"[❌] Failed to send data (HTTP {r.status_code}) - {r.text}")

# stepper_running = False
# TEMP_THRESHOLD = 25.0

# try:
#     while True:
#         current_time = time.strftime("%H:%M:%S")
#         raw_temp = read_raw_temperature()
#         temp_c = compensate_temperature(raw_temp)

#         if temp_c is not None:
#             print(f"[⏱] Time: {current_time}")
#             print(f"[🌡] Temperature: {temp_c}°C")

#         if wiringpi.digitalRead(BUTTON_PIN) == wiringpi.LOW:
#             rotate_clockwise(700 * 2)
#             send_data(temp_c, "Door opened")
#             print("[🔓] Door opened")

#         if temp_c is not None and temp_c > TEMP_THRESHOLD and not stepper_running:
#             print("[⚙] High Temp! Starting Stepper Motor...")
#             stepper_running = True
#             threading.Thread(target=rotate_clockwise, args=(700 * 2,), daemon=True).start()

#         elif temp_c is not None and temp_c <= TEMP_THRESHOLD and stepper_running:
#             print("[⚙] Temperature normal. Stopping stepper motor...")
#             stepper_running = False
#             stop_motor()

#         time.sleep(1)

# except KeyboardInterrupt:
#     print("\n[🛑] Exiting...")
#     spi.close()
#     stop_motor()
#     print("[🛑] Program terminated")
