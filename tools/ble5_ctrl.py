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

    def callback(device, advertising_data):
        if device is not None and device.name is not None and device.name.startswith("C26_"):
            if device.name not in devMap:
                devMap[device.name] = device
                print(device, advertising_data)
                #stop_event.set()

    async with BleakScanner(callback) as scanner:
        await asyncio.wait_for(stop_event.wait(), timeout=3)

for i in range(10):
    try:
        asyncio.run(scan())
    except Exception as e:
        pass

print(devMap)