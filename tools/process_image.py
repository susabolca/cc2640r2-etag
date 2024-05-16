from PIL import Image
import fire


# resize the image to the given width and height, aspect to fill
def resize_image(input_path: str, width: int, height: int, output_path: str) -> str:
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


if __name__ == "__main__":
    fire.Fire()
