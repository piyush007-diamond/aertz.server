"""
Calendar Booker - Create Appointments in Google Calendar
=========================================================
Create calendar events for appointments with read/write access.

Usage:
    python calendar_booker.py --test
    python calendar_booker.py --date 2026-01-09 --time 14:00 --name "John Doe" --email "john@example.com"
"""

import os
from datetime import datetime, timedelta
from pathlib import Path
from zoneinfo import ZoneInfo

from google.oauth2.credentials import Credentials
from google.auth.transport.requests import Request
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from dotenv import load_dotenv
from cloud_auth import get_credentials as get_cloud_credentials

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SCOPES = [
    'https://www.googleapis.com/auth/calendar',  # Full calendar access for creating events
    'https://www.googleapis.com/auth/spreadsheets',
    'https://www.googleapis.com/auth/gmail.send'
]
CREDENTIALS_FILE = Path(__file__).parent / os.getenv('GOOGLE_CREDENTIALS_FILE', 'credentials.json')
TOKEN_FILE = Path(__file__).parent / os.getenv('GOOGLE_TOKEN_FILE', 'token.json')
CALENDAR_ID = os.getenv('GOOGLE_CALENDAR_ID', 'primary')
ARIZONA_TZ = ZoneInfo('America/Phoenix')

SLOT_DURATION_MINUTES = 60


def get_credentials() -> Credentials:
    """Get credentials using cloud_auth helper (supports env vars)."""
    return get_cloud_credentials()


def get_calendar_service():
    """Get authenticated Google Calendar service with write access."""
    creds = get_credentials()
    return build('calendar', 'v3', credentials=creds)


def create_appointment(
    date_str: str,
    time_str: str,
    patient_name: str,
    email: str = None,
    phone: str = None,
    description: str = None
) -> dict:
    """
    Create a calendar event for an appointment.
    
    Args:
        date_str: Date in YYYY-MM-DD format
        time_str: Time in HH:MM:SS or HH:MM format
        patient_name: Name of the patient
        email: Patient's email (optional)
        phone: Patient's phone number (optional)
        description: Additional description (optional)
        
    Returns:
        Created event dictionary
    """
    service = get_calendar_service()
    
    # Parse date and time
    if len(time_str) == 5:
        time_str += ':00'
    
    slot_time = datetime.strptime(time_str, '%H:%M:%S').time()
    slot_date = datetime.strptime(date_str, '%Y-%m-%d').date()
    
    start_dt = datetime.combine(slot_date, slot_time).replace(tzinfo=ARIZONA_TZ)
    end_dt = start_dt + timedelta(minutes=SLOT_DURATION_MINUTES)
    
    # Build event
    event_summary = f"Appointment – {patient_name}"
    if phone:
        event_summary = f"Appointment – {phone}"
    
    event_description = f"Patient: {patient_name}\n"
    if phone:
        event_description += f"Phone: {phone}\n"
    if email:
        event_description += f"Email: {email}\n"
    if description:
        event_description += f"\n{description}"
    event_description += "\n\n[Auto-recovered by verification system]"
    
    event = {
        'summary': event_summary,
        'description': event_description,
        'start': {
            'dateTime': start_dt.isoformat(),
            'timeZone': 'America/Phoenix',
        },
        'end': {
            'dateTime': end_dt.isoformat(),
            'timeZone': 'America/Phoenix',
        },
    }
    
    # Add attendee if email provided
    if email and email != 'NULL' and '@' in email:
        event['attendees'] = [{'email': email}]
    
    try:
        created_event = service.events().insert(
            calendarId=CALENDAR_ID,
            body=event,
            sendUpdates='none'  # Don't send email notifications through Google
        ).execute()
        
        print(f"✅ Created appointment: {event_summary}")
        print(f"   Date: {date_str} @ {time_str}")
        print(f"   Event ID: {created_event.get('id')}")
        
        return created_event
    except HttpError as e:
        print(f"❌ Failed to create event: {e}")
        return None


def test_connection():
    """Test calendar connection with write access."""
    try:
        service = get_calendar_service()
        calendar = service.calendars().get(calendarId=CALENDAR_ID).execute()
        return True, "Connected"
    except Exception as e:
        return False, str(e)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Create appointments in Google Calendar')
    parser.add_argument('--test', action='store_true', help='Test connection')
    parser.add_argument('--date', type=str, help='Appointment date (YYYY-MM-DD)')
    parser.add_argument('--time', type=str, help='Appointment time (HH:MM)')
    parser.add_argument('--name', type=str, help='Patient name')
    parser.add_argument('--email', type=str, help='Patient email')
    parser.add_argument('--phone', type=str, help='Patient phone')
    
    args = parser.parse_args()
    
    if args.test:
        test_connection()
    elif args.date and args.time and args.name:
        create_appointment(
            date_str=args.date,
            time_str=args.time,
            patient_name=args.name,
            email=args.email,
            phone=args.phone
        )
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
