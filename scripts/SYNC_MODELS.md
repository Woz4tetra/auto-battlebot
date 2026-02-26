# sync_models.py

Downloads model files from a private Google Drive folder to the local `models/` directory using Google OAuth2 authentication.

## Prerequisites

- Python 3.11+
- Project Python dependencies installed (`pip install -e .`)
- Google Drive API credentials (see [First-Time Setup](#first-time-setup))

## Quick Start

```bash
# Download all models (auto-detects desktop vs headless)
python scripts/sync_models.py

# Preview what would be downloaded
python scripts/sync_models.py --dry-run

# Force re-download everything
python scripts/sync_models.py --force

# Download to a custom directory
python scripts/sync_models.py -o /path/to/output
```

## First-Time Setup

The script needs Google Drive API credentials before it can access the private folder. If credentials aren't found, the script will offer to walk you through setup interactively.

You can also run the setup wizard directly:

```bash
python scripts/sync_models.py --setup
```

The wizard guides you through:

1. Creating a Google Cloud project
2. Enabling the Google Drive API
3. Configuring the OAuth consent screen
4. Adding yourself as a test user (required for unverified apps)
5. Creating and downloading OAuth credentials

The downloaded `credentials.json` file is saved to `secrets/credentials.json`. This file is gitignored.

## Authenticating on a Remote / Headless Machine

On machines without a desktop environment (SSH sessions, cloud VMs, Jetson boards), the standard browser-based OAuth flow won't work. There are two options:

### Option A: Headless Authentication (Recommended)

Authenticate directly on the remote machine. The script prints a URL that you open in a browser on any other machine, then you paste back the resulting redirect URL.

```bash
python scripts/sync_models.py --headless
```

The script will:

1. Print a Google sign-in URL
2. Ask you to open it in a browser **on any machine**
3. After you authorize, the browser redirects to `http://localhost:1/...` and shows an error page -- this is expected
4. Copy the **full URL** from the browser's address bar and paste it into the terminal

Headless mode is also auto-detected when no `$DISPLAY` or `$WAYLAND_DISPLAY` environment variable is set, so on a typical SSH session the script will use this flow automatically.

### Option B: Token Transfer

Authenticate on a desktop machine and transfer the token to the remote machine.

On the **local machine** (with a browser):

```bash
# Authenticate normally first
python scripts/sync_models.py

# Then export the token as a base64 string
python scripts/sync_models.py --export-token

# Or export to a file
python scripts/sync_models.py --export-token token.json
```

On the **remote machine**:

```bash
# Import from the base64 string
python scripts/sync_models.py --import-token <paste_base64_string>

# Or copy the file over and import it
scp token.json remote-host:~/token.json
ssh remote-host
python scripts/sync_models.py --import-token ~/token.json
```

Both machines must have the same `secrets/credentials.json` file.

## Token Expiration

Tokens for unverified apps expire after 7 days. When this happens the script will:

1. Attempt to refresh the token automatically
2. If refresh fails, present options to re-authenticate or reconfigure credentials

You can also force re-authentication by deleting the token:

```bash
rm ~/.config/auto-battlebot/token.json
python scripts/sync_models.py
```

## All Options

| Flag                    | Description                                                     |
| ----------------------- | --------------------------------------------------------------- |
| `--dry-run`             | Show what would be downloaded without downloading               |
| `--force`               | Re-download all files even if they already exist locally        |
| `-o`, `--output DIR`    | Download to a custom directory (default: `models/`)             |
| `--setup`               | Run the interactive credentials setup wizard                    |
| `--headless`            | Force headless authentication mode (no browser on this machine) |
| `--export-token [FILE]` | Export saved token as base64 (stdout) or to a file              |
| `--import-token TOKEN`  | Import a token from a base64 string or file path                |

## File Locations

| File              | Location                              | Description                                      |
| ----------------- | ------------------------------------- | ------------------------------------------------ |
| OAuth credentials | `secrets/credentials.json`            | Google Cloud OAuth client secret (gitignored)    |
| Drive folder ID   | `secrets/drive_folder_id`             | Google Drive folder ID to sync from (gitignored) |
| Auth token        | `~/.config/auto-battlebot/token.json` | Saved OAuth token (per-user, not in repo)        |
| Downloaded models | `models/`                             | Synced model files (gitignored)                  |

## Troubleshooting

**"Access blocked" or "access_denied" error**

Your email is not added as a test user. Go to [Google Cloud Console > OAuth Audience](https://console.cloud.google.com/auth/audience), add your email as a test user, and try again.

**"credentials.json not found"**

Run `python scripts/sync_models.py --setup` to create credentials, or manually place your OAuth client JSON at `secrets/credentials.json`.

**"Token refresh failed"**

The saved token has expired or been revoked. The script will prompt you to re-authenticate. On headless machines, use `--headless` or transfer a fresh token with `--export-token` / `--import-token`.

**"No files found in the Google Drive folder"**

Verify the folder ID in `secrets/drive_folder_id` is correct and that your authenticated Google account has access to the folder.
