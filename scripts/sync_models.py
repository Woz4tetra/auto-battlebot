#!/usr/bin/env python3
"""
Sync models from a private Google Drive folder.

Downloads model files from a private Google Drive folder to the local models directory.
Uses OAuth2 authentication - on first run, opens a browser for Google sign-in.

Usage:
    python scripts/sync_models.py
    python scripts/sync_models.py --force      # Re-download all files
    python scripts/sync_models.py --dry-run    # Show what would be downloaded
    python scripts/sync_models.py --setup      # Run credentials setup wizard
"""

import argparse
import json
import sys
import time
import webbrowser
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

# Google Cloud Console URLs
CLOUD_CONSOLE_URL = "https://console.cloud.google.com/"
NEW_PROJECT_URL = "https://console.cloud.google.com/projectcreate"
DRIVE_API_URL = "https://console.cloud.google.com/apis/library/drive.googleapis.com"
OAUTH_CONSENT_URL = "https://console.cloud.google.com/apis/credentials/consent"
OAUTH_TEST_USERS_URL = "https://console.cloud.google.com/auth/audience"
CREDENTIALS_URL = "https://console.cloud.google.com/apis/credentials"


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent.resolve()


def get_models_dir() -> Path:
    """Get the models directory path."""
    return get_project_root() / "models"


def get_secrets_dir() -> Path:
    """Get the secrets directory path."""
    secrets_dir = get_project_root() / "secrets"
    secrets_dir.mkdir(parents=True, exist_ok=True)
    return secrets_dir


def get_credentials_path() -> Path:
    """Get the credentials file from the secrets folder."""
    return get_secrets_dir() / CREDENTIALS_FILE


def get_drive_folder_id() -> str:
    """Get the drive folder ID to sync from."""
    with open(get_secrets_dir() / "drive_folder_id") as file:
        return file.read().strip()


def get_token_path() -> Path:
    """Get the path for storing the OAuth token."""
    token_dir = Path.home() / ".config" / "auto-battlebot"
    token_dir.mkdir(parents=True, exist_ok=True)
    return token_dir / TOKEN_FILE


def print_header(text: str) -> None:
    """Print a formatted header."""
    print()
    print("=" * 60)
    print(f"  {text}")
    print("=" * 60)
    print()


def print_step(step_num: int, total: int, text: str) -> None:
    """Print a formatted step."""
    print(f"\n[Step {step_num}/{total}] {text}")
    print("-" * 50)


def wait_for_user(prompt: str = "Press Enter to continue...") -> None:
    """Wait for user to press Enter."""
    input(f"\n{prompt}")


def open_url(url: str, description: str) -> None:
    """Open a URL in the default browser."""
    print(f"\nOpening: {description}")
    print(f"URL: {url}")
    try:
        webbrowser.open(url)
        print("(Browser window should open automatically)")
    except Exception:
        print("(Could not open browser automatically - please open the URL manually)")


def validate_credentials_file(path: Path) -> tuple[bool, str]:
    """
    Validate that a credentials file is properly formatted.

    Returns:
        Tuple of (is_valid, error_message)
    """
    if not path.exists():
        return False, "File does not exist"

    try:
        with open(path) as f:
            data = json.load(f)

        # Check for required fields in OAuth2 credentials
        if "installed" in data:
            installed = data["installed"]
            required_fields = ["client_id", "client_secret", "auth_uri", "token_uri"]
            missing = [f for f in required_fields if f not in installed]
            if missing:
                return False, f"Missing required fields: {', '.join(missing)}"
            return True, ""
        elif "web" in data:
            return (
                False,
                "This appears to be a 'Web application' credential. Please create a 'Desktop application' credential instead.",
            )
        else:
            return (
                False,
                "Invalid credentials format. Expected 'installed' (Desktop app) credentials.",
            )

    except json.JSONDecodeError as e:
        return False, f"Invalid JSON: {e}"
    except Exception as e:
        return False, f"Error reading file: {e}"


def wait_for_credentials_file(credentials_path: Path) -> bool:
    """
    Wait for the user to place the credentials file.

    Args:
        credentials_path: Path where credentials should be placed

    Returns:
        True if valid credentials were found, False otherwise
    """
    print("\nWaiting for credentials file at:")
    print(f"  {credentials_path}")
    print("\nThe setup will automatically continue when you save the file.")
    print()

    check_interval = 1.0
    last_status = ""

    while True:
        if credentials_path.exists():
            is_valid, error = validate_credentials_file(credentials_path)
            if is_valid:
                print("\nCredentials file detected and validated!")
                return True
            else:
                status = f"File found but invalid: {error}"
                if status != last_status:
                    print(f"\n{status}")
                    print("Please download the correct credentials file.")
                    last_status = status

        # Show a simple progress indicator
        sys.stdout.write("\rWaiting...")
        sys.stdout.flush()

        time.sleep(check_interval)


def run_setup_wizard() -> bool:
    """
    Interactive setup wizard for Google Drive API credentials.

    Returns:
        True if setup was successful, False otherwise
    """
    credentials_path = get_credentials_path()

    # Check if already set up
    if credentials_path.exists():
        is_valid, error = validate_credentials_file(credentials_path)
        if is_valid:
            print("\nCredentials already configured!")
            print(f"Location: {credentials_path}")
            response = input("\nRe-run setup anyway? [y/N]: ").strip().lower()
            if response != "y":
                return True

    print_header("Google Drive API Setup Wizard")

    print("This wizard will help you set up Google Drive API access.")
    print("You'll need a Google account to proceed.")
    print("\nThe setup involves 4 steps:")
    print("  1. Create a Google Cloud project")
    print("  2. Enable the Google Drive API")
    print("  3. Configure OAuth consent screen")
    print("  4. Create and download credentials")

    wait_for_user("Press Enter to begin setup...")

    total_steps = 4

    # Step 1: Create Project
    print_step(1, total_steps, "Create a Google Cloud Project")
    print("If you already have a project you want to use, you can skip this step.")
    print("\nIn the Google Cloud Console:")
    print("  - Click 'CREATE PROJECT'")
    print("  - Enter a project name (e.g., 'auto-battlebot')")
    print("  - Click 'CREATE'")
    print("  - Wait for the project to be created")

    open_url(NEW_PROJECT_URL, "Google Cloud - Create Project")
    wait_for_user("Press Enter when your project is ready...")

    # Step 2: Enable Drive API
    print_step(2, total_steps, "Enable the Google Drive API")
    print("This allows your project to access Google Drive.")
    print("\nOn the API page:")
    print("  - Make sure your project is selected in the top dropdown")
    print("  - Click the 'ENABLE' button")

    open_url(DRIVE_API_URL, "Google Drive API")
    wait_for_user("Press Enter when the API is enabled...")

    # Step 3: Configure OAuth Consent
    print_step(3, total_steps, "Configure OAuth Consent Screen")
    print("This is required before creating credentials.")
    print("\nOn the OAuth consent screen:")
    print("  1. Select 'External' user type, click 'CREATE'")
    print("  2. Fill in required fields:")
    print("       - App name: auto-battlebot (or any name)")
    print("       - User support email: your email")
    print("       - Developer contact email: your email")
    print("  3. Click 'SAVE AND CONTINUE' through any remaining steps")
    print("  4. Complete the wizard and return to the dashboard")
    print()
    print("  Note: You can skip optional fields and leave scopes empty.")

    open_url(OAUTH_CONSENT_URL, "OAuth Consent Screen")
    wait_for_user("Press Enter when OAuth consent is configured...")

    # Step 3b: Add Test Users (critical step)
    print_step(3, total_steps, "Add Test Users (IMPORTANT)")
    print("Since the app is unverified, you MUST add yourself as a test user.")
    print("Without this step, you'll get 'access_denied' errors.")
    print()
    print("  1. In the OAuth consent screen, click 'EDIT APP'")
    print("  2. Scroll down or navigate to 'Test users' section")
    print("  3. Click 'ADD USERS'")
    print("  4. Enter YOUR Google email address (the one you'll sign in with)")
    print("  5. Click 'SAVE'")

    open_url(OAUTH_TEST_USERS_URL, "OAuth Test Users")
    wait_for_user("Press Enter when you've added yourself as a test user...")

    # Step 4: Create Credentials
    print_step(4, total_steps, "Create OAuth Credentials")
    print("Now we'll create the credentials file.")
    print("\nOn the Credentials page:")
    print("  - Click 'CREATE CREDENTIALS' at the top")
    print("  - Select 'OAuth client ID'")
    print("  - Application type: 'Desktop app'")
    print("  - Name: auto-battlebot (or any name)")
    print("  - Click 'CREATE'")
    print("")
    print("A popup will appear with your credentials:")
    print("  - Click 'DOWNLOAD JSON'")
    print(f"  - Save the file as: {credentials_path}")
    print("")
    print("  TIP: You can also download later by clicking the download")
    print("       icon next to your credential in the list.")

    open_url(CREDENTIALS_URL, "Google Cloud Credentials")

    # Wait for the file
    if wait_for_credentials_file(credentials_path):
        print_header("Setup Complete!")
        print("Your Google Drive API credentials are now configured.")
        print(f"\nCredentials saved to: {credentials_path}")
        print("\nYou can now run the sync command:")
        print("  python scripts/sync_models.py")
        return True
    else:
        print("\nSetup incomplete. You can:")
        print(f"  1. Manually save credentials.json to: {credentials_path}")
        print("  2. Run this wizard again: python scripts/sync_models.py --setup")
        return False


def ensure_credentials() -> Path:
    """
    Ensure credentials exist, running setup wizard if needed.

    Returns:
        Path to the credentials file
    """
    credentials_path = get_credentials_path()

    if credentials_path.exists():
        is_valid, error = validate_credentials_file(credentials_path)
        if is_valid:
            return credentials_path
        else:
            print(f"Error: Invalid credentials file: {error}")
            print()

    print("Google Drive API credentials not found.")
    print()
    response = input("Would you like to run the setup wizard? [Y/n]: ").strip().lower()

    if response in ("", "y", "yes"):
        if run_setup_wizard():
            return credentials_path
        else:
            sys.exit(1)
    else:
        print("\nTo set up manually, save your credentials to:")
        print(f"  {credentials_path}")
        print("\nOr run the setup wizard:")
        print("  python scripts/sync_models.py --setup")
        sys.exit(1)


def handle_expired_credentials() -> bool:
    """
    Handle expired or revoked credentials with an interactive wizard.

    Returns:
        True if user wants to re-authenticate, False to exit
    """
    token_path = get_token_path()
    credentials_path = get_credentials_path()

    print_header("Credentials Expired or Revoked")

    print("Your saved authentication token is no longer valid.")
    print("This can happen when:")
    print("  - The token has expired (usually after 7 days for unverified apps)")
    print("  - You revoked access in your Google account settings")
    print("  - The OAuth credentials were regenerated in Google Cloud Console")
    print()

    print("Options:")
    print("  1. Re-authenticate (sign in again with Google)")
    print("  2. Create new OAuth credentials (if the old ones were deleted)")
    print("  3. Exit")
    print()

    while True:
        choice = input("Choose an option [1/2/3]: ").strip()

        if choice == "1":
            # Delete old token and re-authenticate
            if token_path.exists():
                token_path.unlink()
                print(f"\nDeleted old token: {token_path}")
            print("You'll be prompted to sign in again.\n")
            return True

        elif choice == "2":
            # Guide through creating new credentials
            print("\nThis will guide you through creating new OAuth credentials.")
            print("Your old credentials.json will be replaced.\n")

            if credentials_path.exists():
                backup_path = credentials_path.with_suffix(".json.backup")
                credentials_path.rename(backup_path)
                print(f"Backed up old credentials to: {backup_path}")

            if token_path.exists():
                token_path.unlink()
                print(f"Deleted old token: {token_path}")

            # Run the setup wizard
            if run_setup_wizard():
                return True
            else:
                sys.exit(1)

        elif choice == "3":
            print("\nExiting. Run the script again when ready.")
            sys.exit(0)
        else:
            print("Invalid choice. Please enter 1, 2, or 3.")


def handle_auth_error(error: Exception) -> None:
    """Handle authentication errors with helpful guidance."""
    error_str = str(error).lower()

    if "access_denied" in error_str or "access blocked" in error_str:
        print("\n" + "=" * 60)
        print("  ERROR: Access Denied - Test User Not Added")
        print("=" * 60)
        print()
        print("Your Google account is not added as a test user for this app.")
        print("Since the app is unverified, only test users can sign in.")
        print()
        print("To fix this:")
        print(
            "  1. Go to Google Cloud Console > APIs & Services > OAuth consent screen"
        )
        print("  2. Click 'EDIT APP' or go to the 'Test users' section")
        print("  3. Click 'ADD USERS'")
        print("  4. Add your email address and save")
        print("  5. Run this script again")
        print()
        print(f"Direct link: {OAUTH_TEST_USERS_URL}")
        print()
        open_url(OAUTH_TEST_USERS_URL, "OAuth Test Users")
        sys.exit(1)

    elif "invalid_grant" in error_str or "token" in error_str:
        # Token expired or revoked - offer to re-authenticate
        if handle_expired_credentials():
            return  # Will retry authentication
        sys.exit(1)

    else:
        # Unknown error
        print("\n" + "=" * 60)
        print("  ERROR: Authentication Failed")
        print("=" * 60)
        print()
        print(f"Error: {error}")
        print()
        print("Possible solutions:")
        print("  1. Check your internet connection")
        print("  2. Try running with --setup to reconfigure credentials")
        print("  3. Delete the token file and try again:")
        print(f"     rm {get_token_path()}")
        sys.exit(1)


def authenticate() -> Credentials:
    """
    Authenticate with Google Drive API using OAuth2.

    On first run, opens a browser for user consent.
    Subsequent runs use the saved token.
    """
    token_path = get_token_path()
    max_retries = 2

    for attempt in range(max_retries):
        creds = None

        # Load existing token if available
        if token_path.exists():
            try:
                creds = Credentials.from_authorized_user_file(str(token_path), SCOPES)
            except Exception as e:
                print(f"Warning: Could not load saved token: {e}")
                token_path.unlink()
                creds = None

        # If no valid credentials, authenticate
        if not creds or not creds.valid:
            if creds and creds.expired and creds.refresh_token:
                print("Refreshing access token...")
                try:
                    creds.refresh(Request())
                except Exception as e:
                    print(f"\nToken refresh failed: {e}")
                    if handle_expired_credentials():
                        continue  # Retry authentication
                    sys.exit(1)
            else:
                credentials_file = ensure_credentials()

                print(f"Using credentials from: {credentials_file}")
                print("Opening browser for Google sign-in...")

                flow = InstalledAppFlow.from_client_secrets_file(
                    str(credentials_file), SCOPES
                )

                try:
                    creds = flow.run_local_server(port=0)
                except Exception as e:
                    handle_auth_error(e)
                    continue  # Retry if handler returns

            # Save the token for future runs
            with open(token_path, "w") as token_file:
                token_file.write(creds.to_json())
            print(f"Token saved to: {token_path}")

        return creds

    print("Authentication failed after multiple attempts.")
    sys.exit(1)


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
            print(f"  Skipping non-file: {file_name}")
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
        print(
            f"Sync complete: {downloaded} downloaded, {skipped} skipped, {failed} failed"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sync model files from private Google Drive folder",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python scripts/sync_models.py              # Download all models
    python scripts/sync_models.py --force      # Force re-download
    python scripts/sync_models.py --dry-run    # Preview without downloading
    python scripts/sync_models.py --setup      # Run credentials setup wizard
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
    parser.add_argument(
        "--setup",
        action="store_true",
        help="Run the interactive setup wizard for Google Drive API credentials",
    )
    args = parser.parse_args()

    # Run setup wizard if requested
    if args.setup:
        success = run_setup_wizard()
        sys.exit(0 if success else 1)

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
