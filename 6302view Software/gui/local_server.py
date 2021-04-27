import http.server
import socketserver
import asyncio
import datetime
import websockets
import serial
import sys
import time
import argparse
import configparser
from time import sleep
import serial.tools.list_ports

# Supported device IDs
VALID_MCUs = []
VALID_MCUs.append(["Teensy 3.2", "a Teensy", 5824])
VALID_MCUs.append(["Arduino Uno (FTDI chipset)", "an Uno", 10755])
VALID_MCUs.append(["ESP8266 D1 Mini Pro (with CH340 Adapter)", "an ESP8266", 6790])
VALID_MCUs.append(["ESP32 Dev Module Rev C(Si Labs Device)", "an ESP32", 4292])

DEVICE_LIST = ""
for i in range(len(VALID_MCUs)):
    DEVICE_LIST += "({}): {}\n".format(i, VALID_MCUs[i][0])

# Default options
VERBOSE = False
PORT    = 6302
DEV     = 1 # uno

# Command-line arg parser
parser = argparse.ArgumentParser()
parser.formatter_class = argparse.RawTextHelpFormatter
parser.description = \
    "Broadcast Serial communication over WebSockets for 6302view."
parser.add_argument("-d", "--device",
    help="specify microcontroller:\n" + DEVICE_LIST)
parser.add_argument("-p", "--port",
    help="specify websocket port")
parser.add_argument("-v", "--verbose",
    action="store_true",
    help="print up- and downstream bytes to console")
parser.add_argument("-w", "--wizard",
    action="store_true",
    help="change default preferences")
args = parser.parse_args()

# config parser
config = configparser.ConfigParser()
config['PREFS'] = {}
config.read('.preferences')

# wizard

def choose_dev():
    while True:
        print("Choose and Enter the number of the Microcontroller you're using:")
        print(DEVICE_LIST)
        choice = input("")
        try:
            if int(choice) not in range(len(VALID_MCUs)):
                print("Invalid Device Chosen. Try again...\n\n")
                time.sleep(0.5)
            else:
                config['PREFS']['DEV'] = choice
                break
        except Exception as e: 
            print(e)
            print("Invalid Input...\n\n")
            time.sleep(0.5)
    print("\n\n")

def choose_port():
    while True:
        print("Enter the Websocket Port you'd like to use for Python to Browser Communication. Valid choices are 6300-6400.")
        choice = input("")
        try:
            if int(choice) not in list(range(6300,6401)):
                print("Invalid Port Chosen. Try again...\n\n")
                time.sleep(0.5)
            else:
                config['PREFS']['PORT'] = choice
                break
        except:
            print("Invalid Input...\n\n")
            time.sleep(0.5)
    print("\n\n")

def choose_verbosity(): 
    while True:
        print("Verbose mode? (Warning may slow down system when operating at high data rates). Enter 1 for True, 0 for False")
        choice = input("")
        try:
            if int(choice) in [0, 1]:
                config['PREFS']['VERBOSE'] = choice
                break
            else:
                print("Invalid choice. Try again...\n\n")
                time.sleep(0.5)
        except Exception as e:
            print(e)
            print("Invalid Input...\n\n")
            time.sleep(0.5)

if args.wizard:
    print("Welcome to 6302View Configuration.")
    config['PREFS'] = {}
    choose_dev()
    choose_port()
    choose_verbosity()
    with open('.preferences', 'w') as configfile:
        config.write(configfile)
else:
    print("Run with -w flag to set preferences\n")

# get up-to-date preferences
config.read('.preferences')
VERBOSE = config['PREFS'].get('VERBOSE', str(VERBOSE)) == '1'
PORT    = int(config['PREFS'].get('PORT', str(PORT)))
DEV     = int(config['PREFS'].get('DEV', str(DEV)))

# command-line args take precedence over .preference only if specified

# verbosity
VERBOSE = VERBOSE if not args.verbose else True

# check port is valid
if args.port:
    p = -1 if not args.port.isdigit() else int(args.port)
    if not( p >= 6300 and p <= 6400 ):
        print("Please specify a valid port within 6300 and 6400 inclusive.")
        sys.exit(1)
    PORT = p

# check device is valid
if args.device:
    d = -1 if not args.device.isdigit() else int(args.device)
    if not d in range(len(VALID_MCUs)):
        print("Please specify a valid device ID in accordance with below:")
        print(DEVICE_LIST)
        sys.exit(1)
    DEV = d

# Use (actual/commercial) device id
DEVID = VALID_MCUs[DEV][2]
print("Ah running {}. Good taste.".format(VALID_MCUs[DEV][1]))

# print("DEBUG::::")
# for item in [VERBOSE, PORT, DEV, DEVID]:
#    print("{} ({})".format(item, type(item)))

print("Starting...")
time.sleep(0.25) #pause a bit so people feel like it is really starting up

def get_usb_port():
    usb_port = list(serial.tools.list_ports.grep("USB-Serial Controller"))
    if len(usb_port) == 1:
        print("Automatically found USB-Serial Controller: {}".format(usb_port[0].description))
        return usb_port[0].device
    else:
        ports = list(serial.tools.list_ports.comports())
        port_dict = {i:[ports[i],ports[i].vid] for i in range(len(ports))}
        usb_id=None
        for p in port_dict:
            print("{}:   {} (Vendor ID: {})".format(p,port_dict[p][0],port_dict[p][1]))
            print(port_dict[p][1],DEVID)
            if port_dict[p][1]==DEVID: #for Teensy (or one type of Teensy anyways...seems to have changed)
                usb_id = p
        if usb_id== None:
            return False
        else:
            print("Found it")
            print("USB-Serial Controller: Device {}".format(p))
            return port_dict[usb_id][0].device

#use the one below later
def get_usb_ports():
    usb_ports = list(serial.tools.list_ports.grep(""))
    print(usb_ports)
    if len(usb_ports) == 1:
        print("Automatically found USB-Serial Controller: {}".format(usb_ports[0].description))
        return usb_ports[0].device
    else:
        ports = list(serial.tools.list_ports.comports())
        port_dict = {i:[ports[i],ports[i].vid] for i in range(len(ports))}
        usb_id=None
        for p in port_dict:
            print("{}:   {} (Vendor ID: {})".format(p,port_dict[p][0],port_dict[p][1]))
        return port_dict


serial_connected = False
comm_buffer = b""  #messages start with \f and end with \n.

async def connect_serial():
    global ser 
    global serial_connected     
    s = get_usb_port()
    print("USB Port: "+str(s))
    baud = 115200
    if s:
        ser = serial.Serial(port = s, 
            baudrate=115200, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS,
            timeout=0.01) #auto-connects already I guess?
        print(ser)
        print("Serial Connected!")
        if ser.isOpen():
             #print(ser.name + ' is open...')
             serial_connected = True
             return True
    return False

ser = connect_serial()

async def send_down(message):
    global serial_connected
    if not serial_connected:
        await connect_serial()
    try:
        msg = message.encode('ascii')
        ser.write(msg)
        if VERBOSE:
            print("▼", msg)
    except Exception as e:
        print("failing on write")
        ser.close()
        serial_connected = False

async def downlink(websocket):
    while True:
        message = await websocket.recv()
        await send_down(message)

async def uplink(websocket):
    global comm_buffer
    global serial_connected
    while True:
        if not serial_connected:
            print("c");
            await connect_serial()
        try:
            data = ser.read(100) #Larger packets improve efficiency.
            await asyncio.sleep(0.001) #Needed so page->cpu messages go.
        except Exception as e:
            print("failing on read")
            ser.close()
            serial_connected = False
            await asyncio.sleep(1)

        try:
            if VERBOSE and data != b'':
                print("▲", data)
            await websocket.send(data)
        except Exception as e:
            #print(e)
            break
        
async def handler(websocket, path):
    global serial_connected
    if not serial_connected:
        await connect_serial()
    #asyncio.gather(uplink(websocket),downlink(websocket))
    page2mcu = asyncio.ensure_future(downlink(websocket))
    mcu2page = asyncio.ensure_future(uplink(websocket))
    done, pending = await asyncio.wait([page2mcu,mcu2page],return_when=asyncio.FIRST_COMPLETED)
    for task in pending:
        task.cancel()
try:
    server = websockets.serve(handler, "127.0.0.1", PORT)
    asyncio.get_event_loop().run_until_complete(server)
    asyncio.get_event_loop().run_forever()
except KeyboardInterrupt:
    print('\nCtrl-C')
finally:
    server.close()
    asyncio.get_event_loop.close()
