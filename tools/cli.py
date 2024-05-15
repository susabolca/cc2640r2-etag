import argparse
import asyncio
import logging
import time
import fire

from bleak import BleakClient, BleakScanner
from bleak.uuids import normalize_uuid_16, uuid16_dict

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s %(name)s - [%(levelname)s] > %(message)s"
)
logger = logging.getLogger(__name__)
logger.level = logging.DEBUG

name_prefix = "C26_b108"

def _filter_device(device, advertisement_data):
    if not device.name:
        return False
    result = device.name.startswith(name_prefix)
    if result:
        logger.debug(f"found device: {device}, {advertisement_data}")
    return result

async def _read_etag(client):
    host_epoch = int(round(time.time()))

    # read current time
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF1))
    epoch = int.from_bytes(value, byteorder="little", signed=False)

    # read time zone
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF2))
    tz_min = int.from_bytes(value, byteorder="little", signed=True)
    logger.info(
        f"# host ts: {host_epoch}, etag ts: {epoch}, diff ({epoch - host_epoch})s, tz: {tz_min // 60}h"
    )

    # battery
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF3))
    battery = int.from_bytes(value, byteorder="little", signed=False)

    # temperature
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF4))
    temp = int.from_bytes(value, byteorder="little", signed=True)
    logger.info(f"# battery: {battery}mV, temperature: {temp}°C")

    # RTC collaborate
    value = await client.read_gatt_char(normalize_uuid_16(0xFFF5))
    rtc = int.from_bytes(value, byteorder="little", signed=False)
    logger.info(f"# rtc: {rtc}")

async def run_ble_client(timeout=30):
    logger.info("starting scan...")

    device = await BleakScanner.find_device_by_filter(
        filterfunc=_filter_device,
        timeout=timeout,
    )
    if device is None:
        logger.error("could not find device")
        raise Exception("device not found")

    logger.info("connecting to device...")

    async with BleakClient(device) as client:
        logger.info("connected")

        await _read_etag(client)

        logger.info("disconnection...")
        await client.disconnect()

    logger.info("disconnected")


if __name__ == "__main__":
    fire.Fire()
