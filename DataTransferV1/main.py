import machine
from micropython_bmpxxx import bmpxxx
import time
import gps_parser
import os
import sdcard_lib

TRANSMITTER_INITIALIZED = False
GPS_INITIALIZED = False
BMP_INITIALIZED = False
SD_INITIALIZED = False


# Initialize LoRa transmitter
def init_transmitter(tx_pin, rx_pin):
    global TRANSMITTER_INITIALIZED

    try:
        uart_transmitter = machine.UART(0, baudrate=115200, tx=machine.Pin(tx_pin), rx=machine.Pin(rx_pin))
        TRANSMITTER_INITIALIZED = True
        print("UART transmitter initialized successfully")
    except(Exception) as e:
        print(f"Failed to initialize UART transmitter: {e}")
        TRANSMITTER_INITIALIZED = False
    return uart_transmitter

# Initialize GPS module
def init_gps(tx_pin, rx_pin):
    global GPS_INITIALIZED
    try:
        gps_module = machine.UART(1, baudrate=9600, tx=machine.Pin(tx_pin), rx=machine.Pin(rx_pin))
        GPS_INITIALIZED = True
        print("GPS module initialized successfully")
    except:
        print(f"Failed to initialize GPS module")
        GPS_INITIALIZED = False
    return gps_module


# Initialize BMP sensor
def init_bmp(sda_pin, scl_pin):
    global BMP_INITIALIZED
    bmp = None  # Initialize to None in case of failure
    try:
        i2c = machine.I2C(0, sda=machine.Pin(sda_pin), scl=machine.Pin(scl_pin))
        bmp = bmpxxx.BMP390(i2c, address=0x77)  # Specify the correct I2C address
        bmp.sea_level_pressure = 1025.90  # Set sea level pressure for accurate altitude readings
        BMP_INITIALIZED = True
        print("BMP sensor initialized successfully")
    except(Exception) as e:
        print(f"Failed to initialize BMP sensor: {e}")
        BMP_INITIALIZED = False
    
    return bmp


# Initialize SD card
def init_sd_card(spi_id=1, sck_pin=10, mosi_pin=11, miso_pin=8, cs_pin=9):
    """
    Initialize SD card with SPI
    Default pins for Pico (SPI1):
    - SCK (Clock):  GP10
    - MOSI (TX):    GP11
    - MISO (RX):    GP8
    - CS (Select):  GP9
    """
    global SD_INITIALIZED
    try:
        import sdcard_lib
        
        # Initialize SPI
        spi = machine.SPI(spi_id, baudrate=100000, polarity=0, phase=0,
                  sck=machine.Pin(sck_pin), mosi=machine.Pin(mosi_pin), miso=machine.Pin(miso_pin))
        
        # Initialize SD card
        sd = sdcard_lib.SDCard(spi, machine.Pin(cs_pin))
        
        # Mount the filesystem
        os.mount(sd, '/sd')
        
        SD_INITIALIZED = True
        print("SD card initialized successfully")
        with open("/sd/flight_data.txt", "a") as f:
            f.write("\n\nTimestamp#Altitude#Latitude#Longitude#GPS_Time|\n")
        return sd
    except Exception as e:
        print(f"Failed to initialize SD card: {e}")
        SD_INITIALIZED = False
        return None


# Write data to SD card
def write_to_sd(data_line, filename="/sd/flight_data.txt"):
    if not SD_INITIALIZED:
        return False
    
    try:
        # Append data to file (create if doesn't exist)
        with open(filename, 'a') as f:
            f.write(data_line + '|')
        return True
    except Exception as e:
        print(f"Error writing to SD card: {e}")
        return False




"""
Obtain GPS data
NOTE: https://core-electronics.com.au/guides/raspberry-pi-pico/how-to-add-gps-to-a-raspberry-pi-pico/ has the information on parsing GPS data
NOTE: gps_parser returns positive and negative values for ease in calculations. calculations back to latitude/longtitude are a xy plane conversion
"""
def get_gps_data(gps_uart, debug=False):
    try:
        # Collect GPS data as fast as possible (0.3 seconds)
        raw_data = ""
        for _ in range(6):
            if gps_uart.any():
                try:
                    chunk = gps_uart.read()
                    if chunk:
                        raw_data += chunk.decode('utf-8', 'ignore')
                except:
                    pass
            time.sleep(0.05)
        # Parse the collected data
        if len(raw_data) > 0:
            data = gps_parser.parse_gps_data(raw_data)
            
            if debug:
                print(f"GPS Debug: has_fix={data.has_fix}, lat={data.latitude:.6f}, lon={data.longitude:.6f}, sats={data.satellites}")
            
            if data.has_fix:
                return f"{data.latitude:.6f}#{data.longitude:.6f}#{data.time}"
            else:
                if debug:
                    print(f"GPS: No fix yet (Satellites: {data.satellites})")
                return None
        else:
            if debug:
                print("GPS: No data received")
            return None
    except Exception as e:
        print(f"Error reading GPS data: {e}")
        return None





####### INTIALIZATION #######
uart_transmitter = init_transmitter(0,1)
gps = init_gps(4,5)
bmp = init_bmp(12,13)
sd_card = init_sd_card(spi_id=1, sck_pin=10, mosi_pin=11, miso_pin=8, cs_pin=9)

if BMP_INITIALIZED:
    bmp.sea_level_pressure = 1025.90  # Set sea level pressure for accurate altitude readings

def send_cmd(uart:machine.UART, cmd:str, wait_response=True, timeout=0.5, debug=False):
    if debug:
        print(f"Sending: {cmd}")
    uart.write((cmd + "\r\n").encode())
    
    if wait_response:
        time.sleep(timeout)
        if uart.any():
            response = uart.read()
            if response:
                try:
                    decoded = response.decode('utf-8', 'ignore').strip()
                    if debug:
                        print(f"Response: {decoded}")
                    return decoded
                except Exception as e:
                    if debug:
                        print(f"Decode error, raw bytes: {response}")
                    return None
    else:
        time.sleep(0.1)
    return None

# --INIT TRANSMITTER--
print("Initializing LoRa transmitter...")
send_cmd(uart_transmitter, 'AT',wait_response=True, debug=True)
send_cmd(uart_transmitter,'AT+RESET',wait_response=True, debug=True)

send_cmd(uart_transmitter, "AT+ADDRESS=1", wait_response=True, debug=True)
send_cmd(uart_transmitter, "AT+NETWORKID=18", wait_response=True, debug=True) #5
send_cmd(uart_transmitter, "AT+BAND=915000000", wait_response=True, debug=True)
send_cmd(uart_transmitter, "AT+PARAMETER=10,8,1,12", wait_response=True, debug=True) # SF=9, BW=9 (125kHz), CR=1, Preamble=12
print("LoRa transmitter configured.\n")

# --INIT RECEIVER--
# send_cmd(uart_receiver, "AT")
# send_cmd(uart_receiver, "AT+RESET")
# time.sleep(1)
# send_cmd(uart_receiver, "AT+ADDRESS=2")
# send_cmd(uart_receiver, "AT+NETWORKID=5")
# send_cmd(uart_receiver, "AT+BAND=915000000")
# send_cmd(uart_receiver, "AT+PARAMETER=9,7,1,12")



# -- TEST LOOP --
packet=0
while True:
    packet+=1

    # attempt to init transmitter if not initialized
    if not TRANSMITTER_INITIALIZED:
        print("Transmitter not initialized. Retrying initialization...")
        print("Continuing data collection in SD CARD")
        uart_transmitter = init_transmitter(0,1)
    
    # Check if BMP is initialized. If not, then use default of None for parsing
    if BMP_INITIALIZED:
        altitude = bmp.altitude
        altitude = round(altitude, 2)
        pressure = round(bmp.pressure,2)
    else:
        altitude = None
    
    # Get GPS data directly from UART
    if GPS_INITIALIZED:
        gps_data = get_gps_data(gps, debug=True)
    else:
        gps_data = None
    
    
    #transmit data in parsable format: {altitude}#{gpsdata}
    msg = f"{packet}#{pressure}#{altitude}#{gps_data}"

    print(f"[TX] Packet: {packet} | Pressure: {pressure} | Alt: {altitude if altitude is not None else 'None'} | GPS: {gps_data if gps_data else 'No Fix'}")
    
    # Write data to SD card
    sd_line = f"{packet}#{pressure}#{altitude}#{gps_data if gps_data else 'None'}"
    if write_to_sd(sd_line):
        print("    ✓ Logged to SD")
    
    # Send command and check for response
    response = send_cmd(uart_transmitter, f"AT+SEND=2,{len(msg)},{msg}", wait_response=True, timeout=0.3, debug = True)
    if response and ("+OK" in response or response == "OK"):
        print("    ✓ Sent via LoRa")
                
    time.sleep(1)  # Minimal delay - fastest possible (0.25)