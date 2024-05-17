import argparse
import asyncio
from contextlib import contextmanager
import logging
import time
import fire

from bleak import BleakClient, BleakScanner
from bleak.uuids import normalize_uuid_16, uuid16_dict


async def _read_etag(logger, client):
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


async def _set_time(logger, client):
    epoch = int(round(time.time()))
    logger.info(f"setting time: {epoch}")

    # set current time
    await client.write_gatt_char(
        normalize_uuid_16(0xFFF1), epoch.to_bytes(4, byteorder="little")
    )


async def _change_mode(client, mode: int):
    # change mode
    await _do_cmd(client, 2, mode)


EPD_CMD_CLR = 1
EPD_CMD_MODE = 2
EPD_CMD_BUF = 3
EPD_CMD_BUF_CONT = 4
EPD_CMD_LUT = 5
EPD_CMD_RST = 6
EPD_CMD_BW = 7
EPD_CMD_RED = 8
EPD_CMD_DP = 9
EPD_CMD_FILL = 10
EPD_CMD_BUF_PUT = 11
EPD_CMD_BUF_GET = 12
EPD_CMD_SNV_WRITE = 13
EPD_CMD_SNV_READ = 14
EPD_CMD_SAVE_CFG = 15


async def _do_cmd(logger, client, cmd, payload=None):
    data = [cmd]

    if cmd in [EPD_CMD_MODE, EPD_CMD_DP]:
        data.append(payload)
    elif cmd in [EPD_CMD_CLR, EPD_CMD_RST, EPD_CMD_BW, EPD_CMD_RED]:
        # no need payload
        pass
    elif cmd == EPD_CMD_BUF:
        chunk_size = 60
        for i in range(0, len(payload), chunk_size):
            chunk = payload[i : i + chunk_size]
            cmd = cmd if i == 0 else EPD_CMD_BUF_CONT
            # logger.debug(f"sending chunk={i+len(chunk)} of data={len(payload)}")
            await client.write_gatt_char(
                normalize_uuid_16(0xFFFE), bytes([cmd] + chunk)
            )
        return
    else:
        raise Exception(f"unsupported cmd: {cmd}, payload: {payload}")

    logger.debug(f"do cmd: {data}")
    await client.write_gatt_char(normalize_uuid_16(0xFFFE), bytes(data))


async def _upload_image_raw_data(logger, client, bw_data, red_data):
    await _do_cmd(client, EPD_CMD_RST)
    time.sleep(2)

    if bw_data:
        await _do_cmd(client, EPD_CMD_BUF, bw_data)
        await _do_cmd(client, EPD_CMD_BW)
    if red_data:
        await _do_cmd(client, EPD_CMD_BUF, red_data)
        await _do_cmd(client, EPD_CMD_RED)
    # display with lut 0
    await _do_cmd(client, EPD_CMD_DP, 0)

    time.sleep(15)


def _image_to_raw_data(logger, image_path):
    from PIL import Image
    from process_image import resize_image, remap_image

    logger.debug(f"processing image: {image_path}")
    fp = resize_image(image_path, 296, 128)
    logger.debug(f"resized image: {fp}")
    fp = remap_image(fp, dither=Image.Dither.NONE)
    logger.debug(f"remapped image: {fp}")

    img = Image.open(fp).convert("RGB")
    width, height = img.size
    bw, red = [], []

    results = set()
    # Process pixels
    logger.debug(f"generate bw/red data: {width}x{height}")
    for y in range(0, height, 8):
        for x in range(width):
            # logger.debug(f"processing pixel: {x}, {y}")
            for i in range(8):
                r, g, b = img.getpixel((x, y + i))
                results.add((r, g, b))
                # three possibilities: black, white, red
                # black: 0x00, 0x00, 0x00
                # red: 0xff, 0x00, 0x00
                # white: 0xff, 0xff, 0xff
                if r == 0x00 and g == 0x00 and b == 0x00:
                    # black
                    bw.append(0)
                    red.append(0)
                elif r == 0xFF and g == 0x00 and b == 0x00:
                    # red
                    red.append(1)
                    bw.append(1)
                elif r == 0xFF and g == 0xFF and b == 0xFF:
                    # white
                    bw.append(1)
                    red.append(0)
                else:
                    raise Exception(f"invalid pixel: {r}, {g}, {b}")

    # merge every 8 pixels into a byte
    bw = [
        int("".join(str(bit) for bit in bw[i : i + 8]), 2) for i in range(0, len(bw), 8)
    ]
    red = [
        int("".join(str(bit) for bit in red[i : i + 8]), 2)
        for i in range(0, len(red), 8)
    ]
    logger.debug(f"unique colors: {results}")
    return bw, red


class CLI(object):
    def __init__(
        self,
        name_prefix="C26_b108",
        log_level=logging.DEBUG,
        timeout=10,
    ):
        self.name_prefix = name_prefix
        self._logger = self._setup_logger(log_level)
        self.timeout = timeout

    def _filter_device(self, device, advertisement_data):
        if not device.name:
            return False
        result = device.name.startswith(self.name_prefix)
        if result:
            self._logger.debug(f"found device: {device}, {advertisement_data}")
        return result

    def _setup_logger(self, log_level):
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(name)s - [%(levelname)s] > %(message)s",
        )
        logger = logging.getLogger(__name__)
        logger.level = log_level
        return logger

    @contextmanager
    async def _ble_client(self):
        self._logger.info("starting scan...")

        device = await BleakScanner.find_device_by_filter(
            filterfunc=self._filter_device,
            timeout=self.timeout,
        )
        if device is None:
            self._logger.error(f"could not find device with name: {self.name_prefix}")
            raise Exception("device not found")

        self._logger.info("connecting to device...")

        async with BleakClient(device) as client:
            self._logger.info(
                f"connected to: {device.name if device.name else device.address}"
            )
            yield client
            self._logger.info("disconnection...")
            await client.disconnect()

        self._logger.info("disconnected")

    async def read_etag(self):
        async with self._ble_client() as client:
            await _read_etag(self._logger, client)

    async def set_time(self):
        async with self._ble_client() as client:
            await _set_time(self._logger, client)

    async def change_mode(self, mode: int):
        async with self._ble_client() as client:
            await _change_mode(self._logger, client, mode)

    # image coule be one of the following:
    # 1. image file path
    # 2. image url
    async def upload_image(self, image: str):
        async with self._ble_client() as client:
            from process_image import download_image_if_needed

            image_path = download_image_if_needed(image)
            # convert 6608697102119889260_296x152.jpg -dither FloydSteinberg -define dither:diffusion-amount=85% -remap palette.png bmp:output.bmp
            bw, red = _image_to_raw_data(self._logger, image_path)
            await _upload_image_raw_data(self._logger, client, bw, red)


if __name__ == "__main__":
    fire.Fire(CLI)
