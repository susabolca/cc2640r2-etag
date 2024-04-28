import struct
from datetime import datetime
import asyncio
from bleak import BleakScanner

devMap = {}

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print(d, d.metadata)
    BleakScanner.discovered_devices_and_advertisement_data = True


async def scan():
    stop_event = asyncio.Event()

    def mv(v):
        return (v>>8)*1000 + (v&0xff*125//32)

    def ShowDevice(device, advertising_data):
        #print(device, advertising_data)
        name = advertising_data.local_name
        data = advertising_data.service_data['0000fff0-0000-1000-8000-00805f9b34fb']
        #print(data)
        # 6 bytes of macaddress, 
        # follow by unt32 of unix timestamp, unt16 of battery, 1 char of temperature
        macAddr = data[0:6]
        # to string
        mac = ":".join("{:02x}".format(x) for x in macAddr) 
        adv = struct.unpack_from("<IHb", data, 6)
        epoch = adv[0] * 60
        dt = datetime.fromtimestamp(epoch).strftime('%Y-%m-%d %H:%M')
        battery = mv(adv[1])
        temp = adv[2]
        print("%s %s %s %dmV %d°C %d" % (name, mac, dt, battery, temp, advertising_data.rssi))

    def callback(device, advertising_data):
        if device is not None and device.name is not None and device.name.startswith("C26_"):
            if device.name not in devMap:
                devMap[device.name] = device
                ShowDevice(device, advertising_data)
                #stop_event.set()

    async with BleakScanner(callback) as scanner:
        await asyncio.wait_for(stop_event.wait(), timeout=3)

print("NAME       MAC               DATE             BATTRY TEMP RSSI")
for i in range(100):
    try:
        asyncio.run(scan())
    except Exception as e:
        pass
