#!/usr/bin/env python3
"""
Remove Failed Annotations

This script reads the validation state file created by validate_yolo_dataset.py
and removes images and their corresponding annotation files that failed validation.
"""

import os
import json
import shutil
import argparse
from pathlib import Path
from typing import Dict, List, Tuple


def load_validation_state(state_file: Path) -> Dict[str, str]:
    """Load validation state from JSON file."""
    if not state_file.exists():
        raise FileNotFoundError(f"State file not found: {state_file}")
    
    with open(state_file, 'r') as f:
        return json.load(f)


def find_image_annotation_pairs(dataset_path: Path) -> List[Tuple[Path, Path]]:
    """Recursively find all image and annotation pairs in the dataset."""
    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'}
    
    # Find all images
    image_files = []
    for ext in image_extensions:
        image_files.extend(dataset_path.rglob(f"*{ext}"))
        image_files.extend(dataset_path.rglob(f"*{ext.upper()}"))
    
    # Match with annotation files
    pairs = []
    for img_path in sorted(image_files):
        # Try to find corresponding label file
        img_str = str(img_path)
        
        # Check if in an 'images' directory
        if '/images/' in img_str or '\\images\\' in img_str:
            label_str = img_str.replace('/images/', '/labels/').replace('\\images\\', '\\labels\\')
        else:
            label_str = img_str
            
        label_path = Path(label_str).with_suffix('.txt')
        
        if label_path.exists():
            pairs.append((img_path, label_path))
    
    return pairs


def remove_failed_annotations(dataset_path: Path, state_file: Path, 
                              dry_run: bool = False, backup: bool = True):
    """
    Remove image and annotation files that failed validation.
    
    Args:
        dataset_path: Root path of the YOLO dataset
        state_file: Path to validation_state.json file
        dry_run: If True, only print what would be deleted without actually deleting
        backup: If True, move files to a backup directory instead of deleting
    """
    # Load validation state
    print(f"Loading validation state from {state_file}")
    validation_state = load_validation_state(state_file)
    
    # Count statistics
    total_validated = len(validation_state)
    failed_count = sum(1 for v in validation_state.values() if v == 'fail')
    passed_count = sum(1 for v in validation_state.values() if v == 'pass')
    
    print(f"\nValidation Statistics:")
    print(f"  Total validated: {total_validated}")
    print(f"  Passed: {passed_count}")
    print(f"  Failed: {failed_count}")
    
    # Find all image-annotation pairs
    print(f"\nScanning dataset at {dataset_path}")
    all_pairs = find_image_annotation_pairs(dataset_path)
    print(f"Found {len(all_pairs)} image-annotation pairs")
    
    # Create backup directory if needed
    backup_dir = None
    if backup and not dry_run:
        backup_dir = dataset_path / "validation_backup"
        backup_dir.mkdir(exist_ok=True)
        print(f"\nCreated backup directory: {backup_dir}")
    
    # Process failed images
    removed_images = []
    removed_labels = []
    
    for img_path, label_path in all_pairs:
        img_key = str(img_path.relative_to(dataset_path))
        
        # Check if this image failed validation
        if validation_state.get(img_key) == 'fail':
            removed_images.append(img_path)
            removed_labels.append(label_path)
            
            if dry_run:
                print(f"\n[DRY RUN] Would remove:")
                print(f"  Image: {img_path}")
                print(f"  Label: {label_path}")
            else:
                if backup:
                    # Move to backup directory
                    backup_img_path = backup_dir / img_path.relative_to(dataset_path)
                    backup_label_path = backup_dir / label_path.relative_to(dataset_path)
                    
                    # Create parent directories
                    backup_img_path.parent.mkdir(parents=True, exist_ok=True)
                    backup_label_path.parent.mkdir(parents=True, exist_ok=True)
                    
                    # Move files
                    shutil.move(str(img_path), str(backup_img_path))
                    shutil.move(str(label_path), str(backup_label_path))
                    
                    print(f"\n✓ Moved to backup:")
                    print(f"  Image: {img_path.name}")
                    print(f"  Label: {label_path.name}")
                else:
                    # Delete files
                    img_path.unlink()
                    label_path.unlink()
                    
                    print(f"\n✗ Deleted:")
                    print(f"  Image: {img_path.name}")
                    print(f"  Label: {label_path.name}")
    
    # Summary
    print(f"\n{'='*60}")
    print("Summary:")
    print(f"  Images {'that would be' if dry_run else ''} removed: {len(removed_images)}")
    print(f"  Labels {'that would be' if dry_run else ''} removed: {len(removed_labels)}")
    
    if backup and not dry_run:
        print(f"  Backup location: {backup_dir}")
    
    if dry_run:
        print("\n⚠ This was a dry run. No files were actually removed.")
        print("  Run without --dry-run to perform the actual removal.")
    else:
        print("\n✓ Removal complete!")
        
        # Update state file to mark as processed
        processed_state_file = dataset_path / "validation_state_processed.json"
        with open(processed_state_file, 'w') as f:
            json.dump(validation_state, f, indent=2)
        print(f"  Saved processed state to: {processed_state_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Remove failed annotations from YOLO dataset based on validation state'
    )
    parser.add_argument(
        'dataset_path', 
        help='Path to YOLO dataset root directory'
    )
    parser.add_argument(
        '--state-file',
        help='Path to validation_state.json file (default: dataset_path/validation_state.json)'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be deleted without actually deleting'
    )
    parser.add_argument(
        '--no-backup',
        action='store_true',
        help='Delete files instead of moving to backup directory'
    )
    
    args = parser.parse_args()
    
    dataset_path = Path(args.dataset_path)
    
    if not dataset_path.exists():
        print(f"Error: Dataset path does not exist: {dataset_path}")
        return 1
    
    # Determine state file path
    if args.state_file:
        state_file = Path(args.state_file)
    else:
        state_file = dataset_path / "validation_state.json"
    
    if not state_file.exists():
        print(f"Error: State file not found: {state_file}")
        print("Make sure you have run validate_yolo_dataset.py first")
        return 1
    
    # Confirm before proceeding (unless dry run)
    if not args.dry_run:
        print("\n⚠ WARNING: This will remove or backup failed annotations from your dataset!")
        print(f"   Dataset: {dataset_path}")
        print(f"   State file: {state_file}")
        
        if args.no_backup:
            print("   Mode: DELETE (files will be permanently deleted)")
        else:
            print("   Mode: BACKUP (files will be moved to validation_backup/)")
        
        response = input("\nProceed? (yes/no): ")
        if response.lower() != 'yes':
            print("Cancelled.")
            return 0
    
    # Perform removal
    remove_failed_annotations(
        dataset_path=dataset_path,
        state_file=state_file,
        dry_run=args.dry_run,
        backup=not args.no_backup
    )
    
    return 0


if __name__ == '__main__':
    exit(main())
