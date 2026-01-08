"""
Cloud-compatible Google Auth
=============================
Uses environment variable for credentials instead of local file.
For cloud deployment, set GOOGLE_CREDENTIALS_JSON env var with the JSON content.
"""

import os
import json
from pathlib import Path

from google.oauth2.credentials import Credentials
from google.auth.transport.requests import Request
from dotenv import load_dotenv

load_dotenv()


def get_credentials():
    """
    Get Google credentials from environment or local token file.
    
    For cloud: Set GOOGLE_TOKEN_JSON env var with the token.json content
    For local: Uses token.json file
    """
    # Try environment variable first (for cloud)
    token_json = os.getenv('GOOGLE_TOKEN_JSON')
    
    if token_json:
        token_data = json.loads(token_json)
        creds = Credentials.from_authorized_user_info(token_data)
        
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
            # Update env var would need external mechanism
            # For now, just use refreshed creds
        
        return creds
    
    # Fallback to local file
    token_file = Path(__file__).parent / os.getenv('GOOGLE_TOKEN_FILE', 'token.json')
    
    if token_file.exists():
        creds = Credentials.from_authorized_user_file(str(token_file))
        
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
            with open(token_file, 'w') as f:
                f.write(creds.to_json())
        
        return creds
    
    raise FileNotFoundError(
        "No credentials found. Set GOOGLE_TOKEN_JSON env var or create token.json"
    )


def export_token_for_cloud():
    """
    Export local token.json as a single-line JSON for cloud env var.
    Run this locally to get the value for GOOGLE_TOKEN_JSON.
    """
    token_file = Path(__file__).parent / 'token.json'
    
    if token_file.exists():
        with open(token_file, 'r') as f:
            token_data = json.load(f)
        
        # Single line JSON for env var
        token_json = json.dumps(token_data)
        print("Copy this value for GOOGLE_TOKEN_JSON environment variable:")
        print("-" * 60)
        print(token_json)
        print("-" * 60)
        return token_json
    else:
        print("token.json not found. Run authentication first.")
        return None


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--export', action='store_true', help='Export token for cloud')
    
    args = parser.parse_args()
    
    if args.export:
        export_token_for_cloud()
