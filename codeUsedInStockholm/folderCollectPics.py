


import os
import shutil
from hashlib import md5
from PIL import Image
from PIL.ExifTags import TAGS
from datetime import datetime

# Ask user for series type
series = input("Is this 'f' or 'r' series? (type 'f' or 'b' or 's'): ").strip().lower()
while series not in ('f', 'b', 's'):
    series = input("Invalid input. Please type 'f' or 'b' or 's' you moron: ").strip().lower()

# Set up directories
root_dir = '/media/toivo/SU650/Scripts/collectPics'  # <<< CHANGE THIS
target_dir = os.path.join(root_dir, 'images')
os.makedirs(target_dir, exist_ok=True)

# Track seen file hashes
seen_hashes = set()
images_with_dates = []

def file_hash(path):
    with open(path, 'rb') as f:
        return md5(f.read()).hexdigest()

def get_image_datetime(image_path):
    try:
        image = Image.open(image_path)
        exif = image._getexif()
        if not exif:
            return None
        for tag, value in exif.items():
            decoded = TAGS.get(tag)
            if decoded == 'DateTimeOriginal':
                return datetime.strptime(value, '%Y:%m:%d %H:%M:%S')
    except Exception as e:
        print(f"Warning: Could not read EXIF from {image_path}: {e}")
    return None

# Traverse and collect images
for dirpath, _, filenames in os.walk(root_dir):
    for filename in filenames:
        if filename.lower().endswith(('.jpg', '.jpeg')):
            full_path = os.path.join(dirpath, filename)

            # Skip if already in target
            if os.path.abspath(dirpath) == os.path.abspath(target_dir):
                continue

            # Skip duplicates
            hash_val = file_hash(full_path)
            if hash_val in seen_hashes:
                continue
            seen_hashes.add(hash_val)

            # Get capture time
            capture_time = get_image_datetime(full_path)
            if capture_time:
                images_with_dates.append((capture_time, full_path))
            else:
                print(f"Skipping {filename} (no valid EXIF timestamp)")

# Sort images by capture time
images_with_dates.sort()

# Copy and rename
for idx, (dt, img_path) in enumerate(images_with_dates, start=1):
    new_filename = f"{series}_{idx:03d}.jpg"
    shutil.copy2(img_path, os.path.join(target_dir, new_filename))

print(f"Copied {len(images_with_dates)} image(s) in time order to '{target_dir}' as {series}_###.jpg")
