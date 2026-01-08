"""
Scheduled Nightly Appointment Verification
============================================
Runs daily at midnight Arizona time (MST/UTC-7) to verify appointments
created in the last 24 hours.

This script:
1. Calculates the time window (last 24 hours in Arizona time)
2. Fetches appointments created in that window from Supabase
3. Verifies each appointment against Google Calendar
4. Logs MISSED appointments to Google Sheets
5. Saves report to timestamped file

Usage:
    python scheduled_verify.py              # Run verification
    python scheduled_verify.py --dry-run    # Show what would be checked without running
"""

import os
import sys
import json
import logging
from datetime import datetime, timedelta, timezone
from pathlib import Path
from zoneinfo import ZoneInfo

from dotenv import load_dotenv
from supabase import create_client

# Import our verification modules
from google_calendar_client import get_events_for_date, find_matching_event
from google_sheets_client import append_missed_appointments

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SUPABASE_URL = os.getenv('SUPABASE_URL')
SUPABASE_KEY = os.getenv('SUPABASE_ANON_KEY')
TABLE_NAME = os.getenv('APPOINTMENT_TABLE', 'patients')
ARIZONA_TZ = ZoneInfo('America/Phoenix')  # Arizona doesn't observe DST

# Setup logging
LOG_DIR = Path(__file__).parent / 'logs'
LOG_DIR.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_DIR / 'scheduled_verify.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def get_arizona_time_window():
    """
    Get the time window for appointments created in the last 24 hours.
    
    Returns:
        Tuple of (start_time, end_time) as ISO strings in UTC
        - start_time: Previous midnight Arizona time (24 hours ago)
        - end_time: Current midnight Arizona time (now)
    """
    # Get current time in Arizona
    now_arizona = datetime.now(ARIZONA_TZ)
    
    # Get today's midnight in Arizona
    today_midnight_arizona = now_arizona.replace(hour=0, minute=0, second=0, microsecond=0)
    
    # Get yesterday's midnight in Arizona
    yesterday_midnight_arizona = today_midnight_arizona - timedelta(days=1)
    
    # Convert to UTC for Supabase query
    start_utc = yesterday_midnight_arizona.astimezone(timezone.utc)
    end_utc = today_midnight_arizona.astimezone(timezone.utc)
    
    return start_utc.isoformat(), end_utc.isoformat()


def get_appointments_created_in_window(start_time: str, end_time: str) -> list[dict]:
    """
    Fetch appointments created between start_time and end_time.
    
    Args:
        start_time: ISO format timestamp (UTC)
        end_time: ISO format timestamp (UTC)
        
    Returns:
        List of appointment dictionaries
    """
    client = create_client(SUPABASE_URL, SUPABASE_KEY)
    
    response = client.table(TABLE_NAME)\
        .select('*')\
        .gte('created_at', start_time)\
        .lt('created_at', end_time)\
        .execute()
    
    return response.data


def verify_appointment(appointment: dict, events_cache: dict = None) -> dict:
    """Verify a single appointment against Google Calendar."""
    appt_date_str = appointment.get('appointment_date')
    
    if not appt_date_str:
        return {
            'appointment': appointment,
            'status': 'error',
            'notes': 'No appointment_date found'
        }
    
    try:
        appt_date = datetime.strptime(appt_date_str, '%Y-%m-%d').date()
    except ValueError:
        return {
            'appointment': appointment,
            'status': 'error',
            'notes': f'Invalid date format: {appt_date_str}'
        }
    
    # Get events for this date
    if events_cache is not None and appt_date_str in events_cache:
        events = events_cache[appt_date_str]
    else:
        events = get_events_for_date(appt_date)
        if events_cache is not None:
            events_cache[appt_date_str] = events
    
    matching_event = find_matching_event(appointment, events)
    
    if matching_event:
        return {
            'appointment': appointment,
            'status': 'matched',
            'calendar_event': matching_event.get('summary'),
            'notes': f"Matched with: {matching_event.get('summary', 'Unknown')}"
        }
    else:
        return {
            'appointment': appointment,
            'status': 'missing',
            'notes': 'No matching calendar event found'
        }


def run_verification(dry_run: bool = False):
    """Run the nightly verification job."""
    start_time, end_time = get_arizona_time_window()
    
    logger.info("=" * 60)
    logger.info("NIGHTLY APPOINTMENT VERIFICATION")
    logger.info("=" * 60)
    logger.info(f"Arizona Time Window:")
    logger.info(f"  From: {start_time}")
    logger.info(f"  To:   {end_time}")
    
    # Fetch appointments
    logger.info(f"Fetching appointments created in window...")
    appointments = get_appointments_created_in_window(start_time, end_time)
    logger.info(f"Found {len(appointments)} appointment(s)")
    
    if dry_run:
        logger.info("DRY RUN - Would verify these appointments:")
        for appt in appointments:
            logger.info(f"  - {appt.get('name')} @ {appt.get('appointment_date')} {appt.get('appointment_time')}")
        return 0
    
    if not appointments:
        logger.info("No appointments to verify. Exiting.")
        return 0
    
    # Verify each appointment
    results = []
    events_cache = {}
    
    for appt in appointments:
        name = appt.get('name', 'Unknown')
        logger.info(f"Checking: {name} @ {appt.get('appointment_date')} {appt.get('appointment_time')}...")
        
        result = verify_appointment(appt, events_cache)
        results.append(result)
        
        if result['status'] == 'matched':
            logger.info(f"  ✅ FOUND in calendar")
        elif result['status'] == 'missing':
            logger.warning(f"  ❌ NOT FOUND in calendar!")
        else:
            logger.error(f"  ⚠️ ERROR: {result['notes']}")
    
    # Generate summary
    matched = sum(1 for r in results if r['status'] == 'matched')
    missing_results = [r for r in results if r['status'] == 'missing']
    errors = sum(1 for r in results if r['status'] == 'error')
    
    logger.info("-" * 60)
    logger.info("SUMMARY")
    logger.info(f"  Total:   {len(results)}")
    logger.info(f"  Matched: {matched}")
    logger.info(f"  Missing: {len(missing_results)}")
    logger.info(f"  Errors:  {errors}")
    
    # Log MISSED appointments to Google Sheets
    if missing_results:
        logger.info("-" * 60)
        logger.info("LOGGING MISSED APPOINTMENTS TO GOOGLE SHEETS...")
        
        missed_appointments = []
        for r in missing_results:
            appt = r['appointment']
            missed_appointments.append({
                'id': appt.get('id', ''),
                'name': appt.get('name', ''),
                'created_at': appt.get('created_at', ''),
                'appointment_date': appt.get('appointment_date', ''),
                'appointment_time': appt.get('appointment_time', ''),
                'email': appt.get('email', ''),
                'phone_number': appt.get('phone_number', '')
            })
        
        try:
            append_missed_appointments(missed_appointments)
            logger.info(f"  ✅ Added {len(missed_appointments)} missed appointments to Google Sheets")
        except Exception as e:
            logger.error(f"  ❌ Failed to log to Google Sheets: {e}")
    
    # Save local report
    timestamp = datetime.now(ARIZONA_TZ).strftime('%Y-%m-%d_%H%M%S')
    report_path = LOG_DIR / f'report_{timestamp}.json'
    
    report = {
        'generated_at': datetime.now(ARIZONA_TZ).isoformat(),
        'time_window': {
            'start': start_time,
            'end': end_time
        },
        'summary': {
            'total': len(results),
            'matched': matched,
            'missing': len(missing_results),
            'errors': errors
        },
        'results': [
            {
                'id': r['appointment'].get('id'),
                'name': r['appointment'].get('name'),
                'created_at': r['appointment'].get('created_at'),
                'date': r['appointment'].get('appointment_date'),
                'time': r['appointment'].get('appointment_time'),
                'email': r['appointment'].get('email'),
                'phone_number': r['appointment'].get('phone_number'),
                'status': r['status'],
                'notes': r.get('notes', '')
            }
            for r in results
        ]
    }
    
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)
    
    logger.info(f"Local report saved: {report_path}")
    logger.info("=" * 60)
    
    # Return exit code based on results
    if len(missing_results) > 0:
        return 1  # Exit with error if any missing
    return 0


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Scheduled nightly appointment verification')
    parser.add_argument('--dry-run', action='store_true', help='Show what would be checked without running')
    
    args = parser.parse_args()
    
    try:
        exit_code = run_verification(dry_run=args.dry_run)
        sys.exit(exit_code if exit_code else 0)
    except Exception as e:
        logger.exception(f"Verification failed: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
