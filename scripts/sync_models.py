#!/usr/bin/env python3
"""
Sync models from a private Google Drive folder.

Downloads model files from a private Google Drive folder to the local models directory.
Uses OAuth2 authentication - on first run, opens a browser for Google sign-in.

Setup:
    1. Go to https://console.cloud.google.com/
    2. Create a project (or select an existing one)
    3. Enable the Google Drive API
    4. Create OAuth2 credentials (Desktop application)
    5. Download the credentials JSON file
    6. Save it as 'credentials.json' in the project root or models/ directory

Usage:
    python scripts/sync_models.py
    python scripts/sync_models.py --force      # Re-download all files
    python scripts/sync_models.py --dry-run    # Show what would be downloaded
"""

import argparse
import sys
from pathlib import Path

from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.http import MediaIoBaseDownload

# OAuth2 scopes - read-only access to Drive
SCOPES = ["https://www.googleapis.com/auth/drive.readonly"]

# Credentials file names
CREDENTIALS_FILE = "credentials.json"
TOKEN_FILE = "token.json"


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent.resolve()


def get_models_dir() -> Path:
    """Get the models directory path."""
    return get_project_root() / "models"


def get_credentials_path() -> Path | None:
    """Get the credentials file from the secrets folder."""
    project_root = get_project_root()
    return project_root / "secrets" / CREDENTIALS_FILE

def get_drive_folder_id() -> str:
    """Get the drive folder ID to sync from."""
    project_root = get_project_root()
    with open(project_root / "secrets" / "drive_folder_id") as file:
        return file.read()


def get_token_path() -> Path:
    """Get the path for storing the OAuth token."""
    token_dir = Path.home() / ".config" / "auto-battlebot"
    token_dir.mkdir(parents=True, exist_ok=True)
    return token_dir / TOKEN_FILE


def authenticate() -> Credentials:
    """
    Authenticate with Google Drive API using OAuth2.

    On first run, opens a browser for user consent.
    Subsequent runs use the saved token.
    """
    creds = None
    token_path = get_token_path()

    # Load existing token if available
    if token_path.exists():
        creds = Credentials.from_authorized_user_file(str(token_path), SCOPES)

    # If no valid credentials, authenticate
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            print("Refreshing access token...")
            creds.refresh(Request())
        else:
            credentials_file = get_credentials_path()
            if not credentials_file:
                print("Error: credentials.json not found!")
                print("\nTo set up authentication:")
                print("  1. Go to https://console.cloud.google.com/")
                print("  2. Create or select a project")
                print("  3. Enable the Google Drive API")
                print("  4. Go to Credentials > Create Credentials > OAuth client ID")
                print("  5. Select 'Desktop application'")
                print("  6. Download the JSON file")
                print(f"  7. Save it as '{CREDENTIALS_FILE}' in the project root")
                sys.exit(1)

            print(f"Using credentials from: {credentials_file}")
            print("Opening browser for Google sign-in...")

            flow = InstalledAppFlow.from_client_secrets_file(
                str(credentials_file), SCOPES
            )
            creds = flow.run_local_server(port=0)

        # Save the token for future runs
        with open(token_path, "w") as token_file:
            token_file.write(creds.to_json())
        print(f"Token saved to: {token_path}")

    return creds


def list_files_in_folder(service, folder_id: str) -> list[dict]:
    """List all files in a Google Drive folder."""
    files = []
    page_token = None

    while True:
        response = (
            service.files()
            .list(
                q=f"'{folder_id}' in parents and trashed = false",
                spaces="drive",
                fields="nextPageToken, files(id, name, mimeType, size)",
                pageToken=page_token,
            )
            .execute()
        )

        files.extend(response.get("files", []))
        page_token = response.get("nextPageToken")

        if not page_token:
            break

    return files


def download_file(service, file_id: str, file_name: str, output_path: Path) -> bool:
    """Download a file from Google Drive."""
    try:
        request = service.files().get_media(fileId=file_id)
        file_path = output_path / file_name

        with open(file_path, "wb") as f:
            downloader = MediaIoBaseDownload(f, request)
            done = False
            while not done:
                status, done = downloader.next_chunk()
                if status:
                    print(f"  Downloading {file_name}: {int(status.progress() * 100)}%")

        return True
    except Exception as e:
        print(f"  Error downloading {file_name}: {e}")
        return False


def format_size(size_bytes: int) -> str:
    """Format file size in human-readable format."""
    for unit in ["B", "KB", "MB", "GB"]:
        if size_bytes < 1024:
            return f"{size_bytes:.1f} {unit}"
        size_bytes /= 1024
    return f"{size_bytes:.1f} TB"


def sync_models(
    output_dir: Path,
    force: bool = False,
    dry_run: bool = False,
) -> None:
    """
    Sync models from Google Drive folder.

    Args:
        output_dir: Directory to download models to.
        force: If True, re-download all files even if they exist.
        dry_run: If True, only show what would be downloaded.
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    drive_folder_id = get_drive_folder_id()

    print(f"Syncing models from Google Drive folder: {drive_folder_id}")
    print(f"Destination: {output_dir}")
    print()

    if dry_run:
        print("[DRY RUN MODE]")
        print()

    # Authenticate and build service
    creds = authenticate()
    service = build("drive", "v3", credentials=creds)

    # List files in the folder
    print("Fetching file list...")
    files = list_files_in_folder(service, drive_folder_id)

    if not files:
        print("No files found in the Google Drive folder.")
        return

    print(f"Found {len(files)} file(s):\n")

    downloaded = 0
    skipped = 0
    failed = 0

    for file_info in files:
        file_name = file_info["name"]
        file_id = file_info["id"]
        mime_type = file_info.get("mimeType", "unknown")
        size = int(file_info.get("size", 0))

        # Skip Google Docs native formats (they need export, not download)
        if mime_type.startswith("application/vnd.google-apps"):
            print(f"  Skipping Google Doc: {file_name}")
            skipped += 1
            continue

        local_path = output_dir / file_name
        size_str = format_size(size) if size else "unknown size"

        if local_path.exists() and not force:
            print(f"  [SKIP] {file_name} ({size_str}) - already exists")
            skipped += 1
            continue

        if dry_run:
            print(f"  [WOULD DOWNLOAD] {file_name} ({size_str})")
            continue

        print(f"  Downloading: {file_name} ({size_str})")
        if download_file(service, file_id, file_name, output_dir):
            downloaded += 1
        else:
            failed += 1

    print()
    if dry_run:
        print(f"Dry run complete. {len(files) - skipped} file(s) would be downloaded.")
    else:
        print(f"Sync complete: {downloaded} downloaded, {skipped} skipped, {failed} failed")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sync model files from private Google Drive folder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Setup (first time only):
    1. Go to https://console.cloud.google.com/
    2. Create a project and enable the Google Drive API
    3. Create OAuth2 credentials (Desktop application)
    4. Download credentials.json to the project root

Examples:
    python scripts/sync_models.py              # Download all models
    python scripts/sync_models.py --force      # Force re-download
    python scripts/sync_models.py --dry-run    # Preview without downloading
    python scripts/sync_models.py -o ./custom  # Download to custom directory
""",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        default=None,
        help="Output directory for models (default: models/ in project root)",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force re-download all files even if they exist",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be downloaded without actually downloading",
    )
    args = parser.parse_args()

    # Determine output directory
    if args.output:
        output_dir = Path(args.output).resolve()
    else:
        output_dir = get_models_dir()

    sync_models(
        output_dir=output_dir,
        force=args.force,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    main()
