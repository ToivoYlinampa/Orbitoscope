from pathlib import Path
import shutil
import re

def extract_index(filename):
    """Extracts the numerical index from a filename."""
    match = re.search(r'(\d+)', filename.stem)
    return int(match.group(1)) if match else -1

def process_folder(folder):
    initial_image = list(folder.glob("initial_image.jpg"))
    backward_images = sorted([f for f in folder.glob("backward_image_*.jpg") if "detailed" not in f.stem], key=extract_index)
    backward_detailed = sorted(folder.glob("backward_image_detailed_*.jpg"), key=extract_index)
    forward_images = sorted(folder.glob("forward_image_*.jpg"), key=extract_index)
    forward_search_images = sorted(folder.glob("forward_search_image_*.jpg"), key=extract_index)

    files_to_delete = set()
    files_to_keep = set()

    # Always delete the initial image unless there is only one backward_image
    if len(backward_images) > 1:
        files_to_delete.update(initial_image)
    else:
        files_to_keep.update(initial_image)

    if forward_images:  # Case: Forward images exist
        print(f"Processing folder with forward images")

        # Delete all backward images
        files_to_delete.update(backward_images + backward_detailed)

        # Keep the three highest indices of forward_search_images
        if len(forward_search_images) > 3:
            files_to_delete.update(forward_search_images[:-3])
            files_to_keep.update(forward_search_images[-3:])
        else:
            files_to_keep.update(forward_search_images)

        # Keep all forward images except the eight highest indices
        if len(forward_images) > 8:
            files_to_keep.update(forward_images[:-8])
            files_to_delete.update(forward_images[-8:])
        else:
            files_to_keep.update(forward_images)
    else:  # Case: No forward images
        print(f"Processing folder with no forward images")

        # Keep the three highest indices of backward_images
        if len(backward_images) > 3:
            print(f"Backward images before deletion: {backward_images}")
            files_to_keep.update(backward_images[-3:])
            files_to_delete.update(backward_images[:-3])
            print(f"Backward images to keep: {backward_images[-3:]}")
            print(f"Backward images to delete: {backward_images[:-3]}")
        else:
            files_to_keep.update(backward_images)

        # Keep all backward_detailed except the eight highest indices
        if len(backward_detailed) > 8:
            print(f"Backward detailed before deletion: {backward_detailed}")
            files_to_keep.update(backward_detailed[:-8])
            files_to_delete.update(backward_detailed[-8:])
            print(f"Backward detailed to keep: {backward_detailed[:-8]}")
            print(f"Backward detailed to delete: {backward_detailed[-8:]}")
        else:
            files_to_keep.update(backward_detailed)

    # Delete unwanted files
    for file in files_to_delete:
        file.unlink()

    # Move kept files one folder up
    for file in files_to_keep:
        shutil.move(str(file), str(folder.parent / file.name))

def main():
    base_folder = Path('.')
    
    for cstack_folder in base_folder.glob('CStack_*'):
        raw_folder = cstack_folder / 'CStack_RAW'
        
        if raw_folder.exists() and raw_folder.is_dir():
            process_folder(raw_folder)
            
            # Delete the CStack_RAW folder after moving the kept files
            shutil.rmtree(raw_folder)

if __name__ == "__main__":
    main()