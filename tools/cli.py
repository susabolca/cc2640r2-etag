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

name_prefix = "C26_b102e6"

def filter_device(device, advertisement_data):
    if not device.name:
        return False
    result = device.name.startswith(name_prefix)
    if result:
        logger.debug(f"found device: {device}, {advertisement_data}")
    return result


async def run_ble_client(timeout=30):
    logger.info("starting scan...")

    device = await BleakScanner.find_device_by_filter(
        filterfunc=filter_device,
        timeout=timeout,
    )
    if device is None:
        logger.error("could not find device")
        raise Exception("device not found")

    logger.info("connecting to device...")

    async with BleakClient(device) as client:
        logger.info("connected")

        host_epoch = int(round(time.time()))

        # read current time
        value = await client.read_gatt_char(normalize_uuid_16(0xFFF1))
        epoch = int.from_bytes(value, byteorder="little", signed=False)

        # read time zone
        value = await client.read_gatt_char(normalize_uuid_16(0xFFF2))
        tz_min = int.from_bytes(value, byteorder="little", signed=True)
        logger.info(f"# host time: {host_epoch}, diff ({epoch - host_epoch}) seconds.")
        logger.info(f"# etag time: {epoch}, tz: {tz_min} minutes of UTC.")

        # # Ensure the device's service list is fully populated
        # await client.get_services()

        # logger.debug("reading characteristics...")

        # # read current time
        # chr = await epd_service.get_characteristic("fff1").read_value()
        # epoch = int.from_bytes(chr, byteorder="little", signed=False)

        # # read time zone
        # chr = await epd_service.get_characteristic("fff2").read_value()
        # tz_min = int.from_bytes(chr, byteorder="little", signed=True)

        # host_epoch = int(round(time.time()))
        # logger.info(f"# host time: {host_epoch}, diff ({epoch - host_epoch}) seconds.")
        # logger.info(f"# etag time: {epoch}, tz: {tz_min} minutes of UTC.")

        # # battery
        # chr = await epd_service.get_characteristic("fff3").read_value()
        # batt = int.from_bytes(chr, byteorder="little", signed=False)

        # # Temperature
        # chr = await epd_service.get_characteristic("fff4").read_value()
        # temp = int.from_bytes(chr, byteorder="little", signed=True)
        # logger.info(f"# etag sensor: battery({batt}mv), temperature({temp}'C).")

        # # RTC Collaborate
        # chr = await epd_service.get_characteristic("fff5").read_value()
        # rtc_collab = int.from_bytes(chr, byteorder="little", signed=True)
        # logger.info(f"# rtc collab: {rtc_collab} every 1 second.")

        logger.info("disconnection...")
        await client.disconnect()

    logger.info("disconnected")


if __name__ == "__main__":
    fire.Fire()
