import argparse
import random
import shutil
from pathlib import Path


def make_split_structure(dataset_root: Path) -> None:
    for subdir in ("train", "val", "test"):
        for subsubdir in ("images", "labels"):
            subdir_path = dataset_root / subdir / subsubdir
            shutil.rmtree(subdir_path, ignore_errors=True)
            subdir_path.mkdir(parents=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("images", help="Path to images")
    parser.add_argument("labels", help="Path to labels")
    parser.add_argument("output", help="Output path")
    parser.add_argument("-t", "--train", type=float, help="Train percentage", default=0.85)
    parser.add_argument("-v", "--val", type=float, help="Validation percentage", default=0.1)
    parser.add_argument("-n", "--num-images", type=int, help="Number of images to include in the dataset", default=None)
    args = parser.parse_args()

    output_path = Path(args.output)
    image_path = Path(args.images)
    label_path = Path(args.labels)
    train_percent = args.train
    val_percent = args.val
    num_images = args.num_images

    make_split_structure(output_path)

    all_image_paths = list(image_path.glob("*.jpg"))
    all_label_paths = list(label_path.glob("*.txt"))
    selected_images = len(all_image_paths) if num_images is None else num_images

    stem_to_annotation_map = {path.stem: path for path in all_label_paths}

    print(f"Splitting up {selected_images} images")
    random.shuffle(all_image_paths)

    num_train = int(train_percent * selected_images)
    num_val = int(val_percent * selected_images)

    datasets = {
        "train": all_image_paths[0:num_train],
        "val": all_image_paths[num_train : num_train + num_val],
        "test": all_image_paths[num_train + num_val : selected_images],
    }

    for subdir_key, sub_dataset in datasets.items():
        for image_path in sub_dataset:
            stem = image_path.stem
            if stem not in stem_to_annotation_map:
                print(f"Skipping {stem}. No annotation.")
                continue
            annotation_path = stem_to_annotation_map[stem]
            shutil.copyfile(image_path, output_path / subdir_key / "images" / image_path.name)
            shutil.copyfile(annotation_path, output_path / subdir_key / "labels" / annotation_path.name)


if __name__ == "__main__":
    main()
