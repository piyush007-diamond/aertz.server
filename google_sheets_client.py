"""
Google Sheets Client for Appointment Reports
==============================================
Handles writing missed appointments to Google Sheets.

The sheet will have columns:
- Date Checked
- Appointment ID  
- Name
- Created At
- Appointment Date
- Appointment Time
- Email
- Phone Number
- Status
"""

import os
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from google.oauth2.credentials import Credentials
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from dotenv import load_dotenv
from cloud_auth import get_credentials as get_cloud_credentials

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
TOKEN_FILE = Path(__file__).parent / os.getenv('GOOGLE_TOKEN_FILE', 'token.json')
SPREADSHEET_ID = os.getenv('GOOGLE_SHEET_ID', '')
ARIZONA_TZ = ZoneInfo('America/Phoenix')

# Sheet headers
HEADERS = [
    'Date Checked',
    'Appointment ID',
    'Name', 
    'Created At',
    'Appointment Date',
    'Appointment Time',
    'Email',
    'Phone Number',
    'Status'
]


def get_sheets_service():
    """Get authenticated Google Sheets service using cloud_auth helper."""
    creds = get_cloud_credentials()
    return build('sheets', 'v4', credentials=creds)


def create_spreadsheet(title: str = "Missed Appointments Report") -> str:
    """Create a new spreadsheet and return its ID."""
    service = get_sheets_service()
    
    spreadsheet = {
        'properties': {'title': title},
        'sheets': [{
            'properties': {'title': 'Report'}
        }]
    }
    
    result = service.spreadsheets().create(body=spreadsheet).execute()
    spreadsheet_id = result.get('spreadsheetId')
    
    # Add headers
    service.spreadsheets().values().update(
        spreadsheetId=spreadsheet_id,
        range='Report!A1:I1',
        valueInputOption='RAW',
        body={'values': [HEADERS]}
    ).execute()
    
    print(f"✅ Created new spreadsheet: {title}")
    print(f"   URL: https://docs.google.com/spreadsheets/d/{spreadsheet_id}")
    print(f"   ID: {spreadsheet_id}")
    print(f"\n   Add this to your .env file:")
    print(f"   GOOGLE_SHEET_ID={spreadsheet_id}")
    
    return spreadsheet_id


def get_or_create_spreadsheet() -> str:
    """Get existing spreadsheet ID or create a new one."""
    sheet_id = os.getenv('GOOGLE_SHEET_ID', '')
    
    if sheet_id:
        # Verify it exists
        try:
            service = get_sheets_service()
            service.spreadsheets().get(spreadsheetId=sheet_id).execute()
            return sheet_id
        except HttpError:
            print(f"Warning: Spreadsheet {sheet_id} not found, creating new one...")
    
    return create_spreadsheet()


def append_missed_appointments(missed_appointments: list[dict]):
    """
    Append missed appointments to the Google Sheet.
    
    Args:
        missed_appointments: List of appointment dictionaries with fields:
            - id, name, created_at, appointment_date, appointment_time, email, phone_number
    """
    if not missed_appointments:
        print("No missed appointments to log.")
        return
    
    spreadsheet_id = get_or_create_spreadsheet()
    service = get_sheets_service()
    
    check_date = datetime.now(ARIZONA_TZ).strftime('%Y-%m-%d %H:%M:%S')
    
    rows = []
    for appt in missed_appointments:
        rows.append([
            check_date,
            str(appt.get('id', '')),
            str(appt.get('name', '')),
            str(appt.get('created_at', '')),
            str(appt.get('appointment_date', '')),
            str(appt.get('appointment_time', '')),
            str(appt.get('email', '')),
            str(appt.get('phone_number', '')),
            'MISSING'
        ])
    
    # Append rows
    service.spreadsheets().values().append(
        spreadsheetId=spreadsheet_id,
        range='Report!A:I',
        valueInputOption='USER_ENTERED',
        insertDataOption='INSERT_ROWS',
        body={'values': rows}
    ).execute()
    
    print(f"✅ Added {len(rows)} missed appointment(s) to Google Sheet")
    print(f"   Sheet URL: https://docs.google.com/spreadsheets/d/{spreadsheet_id}")


def test_connection():
    """Test Google Sheets connection and print status."""
    try:
        spreadsheet_id = get_or_create_spreadsheet()
        service = get_sheets_service()
        result = service.spreadsheets().get(spreadsheetId=spreadsheet_id).execute()
        return True, f"Connected to {result.get('properties', {}).get('title', 'Unknown')}"
    except Exception as e:
        return False, str(e)


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Google Sheets Client')
    parser.add_argument('--test', action='store_true', help='Test connection and create sheet')
    
    args = parser.parse_args()
    
    if args.test:
        test_connection()
