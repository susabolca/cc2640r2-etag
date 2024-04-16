import asyncio
from bleak import BleakScanner

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        print(d, d.metadata)
    BleakScanner.discovered_devices_and_advertisement_data = True

asyncio.run(main())