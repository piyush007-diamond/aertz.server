"""
Slot Checker - Check Google Calendar Availability
==================================================
Check if specific time slots are available in Google Calendar.

Usage:
    python slot_checker.py --date 2026-01-09 --time 14:00
    python slot_checker.py --date 2026-01-09 --available
"""

import os
from datetime import datetime, timedelta
from pathlib import Path
from zoneinfo import ZoneInfo

from google.oauth2.credentials import Credentials
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from dotenv import load_dotenv

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
TOKEN_FILE = Path(__file__).parent / os.getenv('GOOGLE_TOKEN_FILE', 'token.json')
CALENDAR_ID = os.getenv('GOOGLE_CALENDAR_ID', 'primary')
ARIZONA_TZ = ZoneInfo('America/Phoenix')

# Working hours (Arizona time)
WORKING_START_HOUR = 9   # 9 AM
WORKING_END_HOUR = 17    # 5 PM
SLOT_DURATION_MINUTES = 60  # 1 hour slots


def get_calendar_service():
    """Get authenticated Google Calendar service."""
    if not TOKEN_FILE.exists():
        raise FileNotFoundError(f"Token file not found: {TOKEN_FILE}")
    
    creds = Credentials.from_authorized_user_file(str(TOKEN_FILE))
    
    if creds and creds.expired and creds.refresh_token:
        creds.refresh(Request())
        with open(TOKEN_FILE, 'w') as token:
            token.write(creds.to_json())
    
    return build('calendar', 'v3', credentials=creds)


def get_events_for_date(date_str: str) -> list[dict]:
    """
    Fetch all calendar events for a specific date.
    
    Args:
        date_str: Date in YYYY-MM-DD format
        
    Returns:
        List of event dictionaries
    """
    service = get_calendar_service()
    
    date = datetime.strptime(date_str, '%Y-%m-%d').date()
    
    # Create time bounds for the day in Arizona time
    start_dt = datetime.combine(date, datetime.min.time()).replace(tzinfo=ARIZONA_TZ)
    end_dt = start_dt + timedelta(days=1)
    
    try:
        events_result = service.events().list(
            calendarId=CALENDAR_ID,
            timeMin=start_dt.isoformat(),
            timeMax=end_dt.isoformat(),
            singleEvents=True,
            orderBy='startTime'
        ).execute()
        
        return events_result.get('items', [])
    except HttpError as e:
        print(f"Error fetching events: {e}")
        return []


def parse_event_times(event: dict) -> tuple:
    """Extract start and end times from a calendar event."""
    start = event.get('start', {})
    end = event.get('end', {})
    
    if 'dateTime' in start:
        start_dt = datetime.fromisoformat(start['dateTime'].replace('Z', '+00:00'))
        end_dt = datetime.fromisoformat(end['dateTime'].replace('Z', '+00:00'))
        return start_dt.astimezone(ARIZONA_TZ), end_dt.astimezone(ARIZONA_TZ)
    
    return None, None


def is_slot_available(date_str: str, time_str: str, duration_minutes: int = SLOT_DURATION_MINUTES) -> bool:
    """
    Check if a specific time slot is available.
    
    Args:
        date_str: Date in YYYY-MM-DD format
        time_str: Time in HH:MM:SS or HH:MM format
        duration_minutes: Duration of the slot in minutes
        
    Returns:
        True if slot is available, False otherwise
    """
    # Parse the requested slot time
    try:
        if len(time_str) == 5:
            time_str += ':00'
        slot_time = datetime.strptime(time_str, '%H:%M:%S').time()
        slot_date = datetime.strptime(date_str, '%Y-%m-%d').date()
        slot_start = datetime.combine(slot_date, slot_time).replace(tzinfo=ARIZONA_TZ)
        slot_end = slot_start + timedelta(minutes=duration_minutes)
    except ValueError as e:
        print(f"Invalid date/time format: {e}")
        return False
    
    # Check working hours
    if slot_time.hour < WORKING_START_HOUR or slot_time.hour >= WORKING_END_HOUR:
        return False
    
    # Check if slot end is within working hours
    if slot_end.hour > WORKING_END_HOUR:
        return False
    
    # Check against existing events
    events = get_events_for_date(date_str)
    
    for event in events:
        event_start, event_end = parse_event_times(event)
        if event_start is None:
            continue  # Skip all-day events or unparseable events
        
        # Check for overlap
        if slot_start < event_end and slot_end > event_start:
            return False
    
    return True


def get_available_slots(date_str: str) -> list[str]:
    """
    Get all available 1-hour slots for a given date.
    
    Args:
        date_str: Date in YYYY-MM-DD format
        
    Returns:
        List of available times in HH:MM format
    """
    available = []
    
    for hour in range(WORKING_START_HOUR, WORKING_END_HOUR):
        time_str = f"{hour:02d}:00:00"
        if is_slot_available(date_str, time_str):
            available.append(f"{hour:02d}:00")
    
    return available


def get_available_slots_json(date_str: str) -> dict:
    """Get available slots in JSON format for the booking page."""
    slots = get_available_slots(date_str)
    return {
        'date': date_str,
        'timezone': 'America/Phoenix',
        'working_hours': f'{WORKING_START_HOUR}:00 - {WORKING_END_HOUR}:00',
        'slot_duration_minutes': SLOT_DURATION_MINUTES,
        'available_slots': slots
    }


def main():
    import argparse
    import json
    
    parser = argparse.ArgumentParser(description='Check Google Calendar slot availability')
    parser.add_argument('--date', type=str, required=True, help='Date to check (YYYY-MM-DD)')
    parser.add_argument('--time', type=str, help='Specific time to check (HH:MM or HH:MM:SS)')
    parser.add_argument('--available', action='store_true', help='List all available slots')
    parser.add_argument('--json', action='store_true', help='Output in JSON format')
    
    args = parser.parse_args()
    
    if args.time:
        available = is_slot_available(args.date, args.time)
        if args.json:
            print(json.dumps({'date': args.date, 'time': args.time, 'available': available}))
        else:
            status = "✅ AVAILABLE" if available else "❌ NOT AVAILABLE"
            print(f"Slot {args.date} @ {args.time}: {status}")
    
    elif args.available:
        if args.json:
            print(json.dumps(get_available_slots_json(args.date), indent=2))
        else:
            slots = get_available_slots(args.date)
            print(f"\n📅 Available slots for {args.date}:")
            print(f"   Working hours: {WORKING_START_HOUR}:00 - {WORKING_END_HOUR}:00 (Arizona)")
            print(f"   Slot duration: {SLOT_DURATION_MINUTES} minutes\n")
            if slots:
                for slot in slots:
                    print(f"   ✅ {slot}")
            else:
                print("   ❌ No available slots")


if __name__ == '__main__':
    main()
