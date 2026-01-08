"""
Google Calendar Client for Appointment Verification
====================================================
Handles Google Calendar API operations.

Usage:
    python google_calendar_client.py --test           # Test connection (will prompt OAuth)
    python google_calendar_client.py --today          # Get today's events
    python google_calendar_client.py --date 2026-01-08  # Get events for specific date
"""

import os
import sys
from datetime import datetime, date, timedelta
from pathlib import Path
from dotenv import load_dotenv

from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SCOPES = [
    'https://www.googleapis.com/auth/calendar.readonly',
    'https://www.googleapis.com/auth/spreadsheets'
]
CREDENTIALS_FILE = Path(__file__).parent / os.getenv('GOOGLE_CREDENTIALS_FILE', 'credentials.json')
TOKEN_FILE = Path(__file__).parent / os.getenv('GOOGLE_TOKEN_FILE', 'token.json')
CALENDAR_ID = os.getenv('GOOGLE_CALENDAR_ID', 'primary')


def get_credentials() -> Credentials:
    """
    Get or refresh Google OAuth credentials.
    Will prompt user for authentication on first run.
    """
    creds = None
    
    # Load existing token if available
    if TOKEN_FILE.exists():
        creds = Credentials.from_authorized_user_file(str(TOKEN_FILE), SCOPES)
    
    # If no valid credentials, prompt for login
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            if not CREDENTIALS_FILE.exists():
                raise FileNotFoundError(
                    f"Credentials file not found: {CREDENTIALS_FILE}\n"
                    "Please download credentials.json from Google Cloud Console"
                )
            flow = InstalledAppFlow.from_client_secrets_file(str(CREDENTIALS_FILE), SCOPES)
            creds = flow.run_local_server(port=0)
        
        # Save credentials for next run
        with open(TOKEN_FILE, 'w') as token:
            token.write(creds.to_json())
    
    return creds


def get_calendar_service():
    """Create and return Google Calendar API service."""
    creds = get_credentials()
    return build('calendar', 'v3', credentials=creds)


def get_events_for_date(target_date: date) -> list[dict]:
    """
    Fetch all calendar events for a specific date.
    
    Args:
        target_date: The date to fetch events for
        
    Returns:
        List of event dictionaries
    """
    service = get_calendar_service()
    
    # Create time bounds for the day
    time_min = datetime.combine(target_date, datetime.min.time()).isoformat() + 'Z'
    time_max = datetime.combine(target_date + timedelta(days=1), datetime.min.time()).isoformat() + 'Z'
    
    try:
        events_result = service.events().list(
            calendarId=CALENDAR_ID,
            timeMin=time_min,
            timeMax=time_max,
            singleEvents=True,
            orderBy='startTime'
        ).execute()
        
        return events_result.get('items', [])
    except HttpError as e:
        print(f"❌ Error fetching events: {e}")
        return []


def get_today_events() -> list[dict]:
    """Fetch all calendar events for today."""
    return get_events_for_date(date.today())


def parse_event_time(event: dict) -> tuple[date, str]:
    """
    Parse event start date and time from Google Calendar event.
    
    Returns:
        Tuple of (date, time_string)
    """
    start = event.get('start', {})
    
    if 'dateTime' in start:
        # Timed event
        dt = datetime.fromisoformat(start['dateTime'].replace('Z', '+00:00'))
        return dt.date(), dt.strftime('%H:%M:%S')
    elif 'date' in start:
        # All-day event
        d = datetime.strptime(start['date'], '%Y-%m-%d').date()
        return d, 'all-day'
    
    return None, None


def format_event(event: dict) -> str:
    """Format a calendar event for display."""
    summary = event.get('summary', 'No Title')
    event_date, event_time = parse_event_time(event)
    description = event.get('description', '')[:50]
    
    return f"  📌 {summary} | {event_date} @ {event_time} | {description}"


def find_matching_event(appointment: dict, events: list[dict], time_tolerance_minutes: int = 30) -> dict | None:
    """
    Find a calendar event that matches the given appointment.
    
    Matching criteria:
    1. Same date (required)
    2. Time within tolerance (default 30 minutes)
    3. Name appears in event summary or description (optional bonus)
    
    Args:
        appointment: Appointment dict from Supabase
        events: List of Google Calendar events
        time_tolerance_minutes: How many minutes difference is acceptable
        
    Returns:
        Matching event dict or None
    """
    appt_date_str = appointment.get('appointment_date')
    appt_time_str = appointment.get('appointment_time')
    appt_name = appointment.get('name', '').lower()
    
    if not appt_date_str or not appt_time_str:
        return None
    
    # Parse appointment date and time
    try:
        appt_date = datetime.strptime(appt_date_str, '%Y-%m-%d').date()
        appt_time = datetime.strptime(appt_time_str, '%H:%M:%S').time()
        appt_datetime = datetime.combine(appt_date, appt_time)
    except ValueError:
        return None
    
    best_match = None
    best_score = 0
    
    for event in events:
        event_date, event_time_str = parse_event_time(event)
        
        if event_date != appt_date:
            continue
        
        if event_time_str == 'all-day':
            # All-day events get low priority match
            score = 1
        else:
            try:
                event_time = datetime.strptime(event_time_str, '%H:%M:%S').time()
                event_datetime = datetime.combine(event_date, event_time)
                
                # Calculate time difference
                time_diff = abs((appt_datetime - event_datetime).total_seconds() / 60)
                
                if time_diff <= time_tolerance_minutes:
                    score = 10 - (time_diff / time_tolerance_minutes * 5)  # Higher score for closer times
                else:
                    continue  # Time difference too large
            except ValueError:
                continue
        
        # Bonus points if name appears in event
        summary = event.get('summary', '').lower()
        description = event.get('description', '').lower()
        
        if appt_name and (appt_name in summary or appt_name in description):
            score += 5
        
        if score > best_score:
            best_score = score
            best_match = event
    
    return best_match


def test_connection() -> bool:
    """Test Google Calendar connection and print status."""
    try:
        service = get_calendar_service()
        # Try to fetch calendar info
        calendar = service.calendars().get(calendarId=CALENDAR_ID).execute()
        print(f"✅ Google Calendar connection successful!")
        print(f"   Calendar: {calendar.get('summary', 'Unknown')}")
        print(f"   Calendar ID: {CALENDAR_ID}")
        
        # Get today's events count
        today_events = get_today_events()
        print(f"   Today's events: {len(today_events)}")
        return True
    except Exception as e:
        print(f"❌ Google Calendar connection failed!")
        print(f"   Error: {e}")
        return False


def main():
    """CLI entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Google Calendar Client')
    parser.add_argument('--test', action='store_true', help='Test connection')
    parser.add_argument('--today', action='store_true', help='Get today\'s events')
    parser.add_argument('--date', type=str, help='Get events for date (YYYY-MM-DD)')
    
    args = parser.parse_args()
    
    if args.test:
        test_connection()
    elif args.today:
        events = get_today_events()
        print(f"\n📅 Calendar events for today ({date.today()}):")
        print(f"   Found {len(events)} event(s)\n")
        for event in events:
            print(format_event(event))
    elif args.date:
        try:
            target_date = datetime.strptime(args.date, '%Y-%m-%d').date()
            events = get_events_for_date(target_date)
            print(f"\n📅 Calendar events for {target_date}:")
            print(f"   Found {len(events)} event(s)\n")
            for event in events:
                print(format_event(event))
        except ValueError:
            print("❌ Invalid date format. Use YYYY-MM-DD")
            sys.exit(1)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
