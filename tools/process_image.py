from PIL import Image
import fire
import os


def generate_temp_output_path(input_path: str, ext: str = None) -> str:
    filename = os.path.basename(input_path)
    name_without_ext, original_ext = os.path.splitext(filename)
    if ext is None:
        ext = original_ext
    if not ext.startswith("."):
        ext = f".{ext}"
    # /tmp/blabla.ext
    import tempfile
    return os.path.join(tempfile.gettempdir(), f"{name_without_ext}{ext}")


# resize the image to the given width and height, aspect to fill
def resize_image(
    input_path: str, width: int, height: int, output_path: str = None
) -> str:
    if output_path is None:
        output_path = generate_temp_output_path(input_path)

    # Open the image file
    with Image.open(input_path) as img:
        # Calculate the required aspect ratio
        desired_aspect_ratio = width / height
        img_aspect_ratio = img.width / img.height

        # Resize and crop logic
        if desired_aspect_ratio < img_aspect_ratio:
            # The requested aspect ratio is wider than the image's aspect ratio
            resize_height = height
            resize_width = int(height * img_aspect_ratio)
        else:
            # The requested aspect ratio is taller than the image's aspect ratio
            resize_width = width
            resize_height = int(width / img_aspect_ratio)

        # Resize the image
        img = img.resize((resize_width, resize_height), Image.LANCZOS)

        # Calculate cropping area
        left = (resize_width - width) / 2
        top = (resize_height - height) / 2
        right = (resize_width + width) / 2
        bottom = (resize_height + height) / 2

        # Crop the image
        img = img.crop((left, top, right, bottom))

        # Save the resized image
        img.save(output_path)

    return output_path


# palette.png generate with below command
# convert -size 1x3 xc:black xc:white xc:red +append palette.png
# fmt: off
PALETTE_BWR =  [
    0, 0, 0,        # black
    255, 255, 255,  # white
    255, 0, 0,      # red
]
# fmt: on


# remap the image to the given palette
# convert input.jpg -dither FloydSteinberg -define dither:diffusion-amount=85% -remap palette.png bmp:output.bmp
def remap_image(
    input_path: str, palette: [int] = None, output_path: str = None,
    dither=Image.Dither.FLOYDSTEINBERG,
    ) -> str:
    if palette is None:
        palette = PALETTE_BWR
    if output_path is None:
        output_path = generate_temp_output_path(input_path, ".bmp")

    with Image.open(input_path) as original_image:
        original_image = original_image.convert("RGB")

        # Create a new image using the 'P' mode (palette-based) with the same
        # dimensions as the original.
        palette_image = Image.new("P", original_image.size)

        # Put the custom palette into the image
        palette_image.putpalette(palette)

        # Convert the original image to 'P' mode with our custom palette.
        # The .quantize() method maps colors to the nearest color in the palette.
        # You can play around with the 'dither' and 'colors' parameters if necessary.
        converted_image = original_image.quantize(palette=palette_image, dither=dither)

        # Save or display your image
        converted_image.save(output_path)

    return output_path

    # If you want to work with the RGB values, convert it back to 'RGB' mode.
    # rgb_image = converted_image.convert("RGB")


if __name__ == "__main__":
    fire.Fire()
