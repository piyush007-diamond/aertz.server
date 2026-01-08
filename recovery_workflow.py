"""
Recovery Workflow - Auto-recover Missed Appointments
======================================================
Runs 30 minutes after verification to:
1. Check if original slots are available
2. Auto-book if available
3. Send recovery emails if not available

Schedule: 1:00 PM IST (12:30 AM Arizona)

Usage:
    python recovery_workflow.py           # Run recovery
    python recovery_workflow.py --dry-run # Preview without changes
"""

import os
import json
import logging
from datetime import datetime, timedelta
from pathlib import Path
from zoneinfo import ZoneInfo

from dotenv import load_dotenv

# Import our modules
from slot_checker import is_slot_available, get_available_slots
from calendar_booker import create_appointment
from email_sender import send_recovery_email
from google_sheets_client import get_sheets_service

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SPREADSHEET_ID = os.getenv('GOOGLE_SHEET_ID', '')
BOOKING_PAGE_URL = os.getenv('BOOKING_PAGE_URL', 'file:///C:/Users/Piyush/Downloads/agents/execution/booking_page/index.html')
ARIZONA_TZ = ZoneInfo('America/Phoenix')

# Setup logging
LOG_DIR = Path(__file__).parent / 'logs'
LOG_DIR.mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_DIR / 'recovery_workflow.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def get_missed_appointments_from_sheet() -> list[dict]:
    """
    Read missed appointments from Google Sheet that need recovery.
    Only gets rows where Status is 'MISSING'.
    """
    if not SPREADSHEET_ID:
        logger.warning("No GOOGLE_SHEET_ID configured. Skipping sheet read.")
        return []
    
    service = get_sheets_service()
    
    try:
        result = service.spreadsheets().values().get(
            spreadsheetId=SPREADSHEET_ID,
            range='Report!A:I'
        ).execute()
        
        rows = result.get('values', [])
        if len(rows) < 2:
            return []
        
        headers = rows[0]
        appointments = []
        
        for i, row in enumerate(rows[1:], start=2):  # Start at 2 to account for header
            # Pad row to match headers
            while len(row) < len(headers):
                row.append('')
            
            row_dict = dict(zip(headers, row))
            
            # Only process rows with MISSING status
            if row_dict.get('Status', '').upper() == 'MISSING':
                row_dict['_row_number'] = i  # Track row for updates
                appointments.append(row_dict)
        
        return appointments
    except Exception as e:
        logger.error(f"Failed to read sheet: {e}")
        return []


def update_sheet_status(row_number: int, new_status: str, notes: str = ''):
    """Update the status of a row in the Google Sheet."""
    if not SPREADSHEET_ID:
        return
    
    service = get_sheets_service()
    
    try:
        # Update Status column (I) and add recovery notes if available
        service.spreadsheets().values().update(
            spreadsheetId=SPREADSHEET_ID,
            range=f'Report!I{row_number}',
            valueInputOption='RAW',
            body={'values': [[new_status]]}
        ).execute()
        
        logger.info(f"Updated row {row_number} status to: {new_status}")
    except Exception as e:
        logger.error(f"Failed to update sheet: {e}")


def process_missed_appointment(appt: dict, dry_run: bool = False) -> str:
    """
    Process a single missed appointment.
    
    Returns:
        Status string: 'RECOVERED', 'EMAIL_SENT', 'NO_EMAIL', 'ERROR'
    """
    name = appt.get('Name', 'Unknown')
    email = appt.get('Email', '')
    phone = appt.get('Phone Number', '')
    appt_date = appt.get('Appointment Date', '')
    appt_time = appt.get('Appointment Time', '')
    row_number = appt.get('_row_number', 0)
    
    logger.info(f"Processing: {name} | {appt_date} @ {appt_time} | {email}")
    
    if not appt_date or not appt_time:
        logger.warning(f"  Missing date/time, skipping")
        return 'ERROR'
    
    # Check if original slot is available
    if is_slot_available(appt_date, appt_time):
        logger.info(f"  ✅ Original slot is available!")
        
        if dry_run:
            logger.info(f"  [DRY RUN] Would create calendar event")
        else:
            # Auto-book the appointment
            result = create_appointment(
                date_str=appt_date,
                time_str=appt_time,
                patient_name=name,
                email=email,
                phone=phone,
                description="Auto-recovered appointment"
            )
            
            if result:
                if row_number:
                    update_sheet_status(row_number, 'RECOVERED')
                return 'RECOVERED'
            else:
                return 'ERROR'
    else:
        logger.info(f"  ❌ Original slot not available")
        
        # Check if we have email to send recovery
        if email and email != 'NULL' and '@' in email:
            logger.info(f"  📧 Sending recovery email to {email}")
            
            if dry_run:
                logger.info(f"  [DRY RUN] Would send email to {email}")
            else:
                success = send_recovery_email(
                    to_email=email,
                    patient_name=name,
                    original_date=appt_date,
                    original_time=appt_time,
                    booking_link=BOOKING_PAGE_URL
                )
                
                if success:
                    if row_number:
                        update_sheet_status(row_number, 'EMAIL_SENT')
                    return 'EMAIL_SENT'
                else:
                    return 'ERROR'
        else:
            logger.warning(f"  ⚠️ No valid email address")
            if row_number and not dry_run:
                update_sheet_status(row_number, 'NO_EMAIL')
            return 'NO_EMAIL'
    
    return 'RECOVERED' if dry_run else 'ERROR'


def run_recovery(dry_run: bool = False):
    """Run the recovery workflow."""
    logger.info("=" * 60)
    logger.info("MISSED APPOINTMENT RECOVERY WORKFLOW")
    logger.info("=" * 60)
    logger.info(f"Time: {datetime.now(ARIZONA_TZ).strftime('%Y-%m-%d %H:%M:%S')} Arizona")
    
    if dry_run:
        logger.info("MODE: DRY RUN (no changes will be made)")
    
    # Get missed appointments from sheet
    logger.info("Fetching missed appointments from Google Sheet...")
    appointments = get_missed_appointments_from_sheet()
    
    if not appointments:
        logger.info("No missed appointments to process.")
        return
    
    logger.info(f"Found {len(appointments)} appointment(s) to process")
    logger.info("-" * 60)
    
    # Process each appointment
    results = {
        'RECOVERED': 0,
        'EMAIL_SENT': 0,
        'NO_EMAIL': 0,
        'ERROR': 0
    }
    
    for appt in appointments:
        status = process_missed_appointment(appt, dry_run)
        results[status] = results.get(status, 0) + 1
    
    # Summary
    logger.info("-" * 60)
    logger.info("RECOVERY SUMMARY")
    logger.info(f"  ✅ Recovered (auto-booked): {results['RECOVERED']}")
    logger.info(f"  📧 Emails sent: {results['EMAIL_SENT']}")
    logger.info(f"  ⚠️ No email available: {results['NO_EMAIL']}")
    logger.info(f"  ❌ Errors: {results['ERROR']}")
    logger.info("=" * 60)
    
    # Save recovery report
    timestamp = datetime.now(ARIZONA_TZ).strftime('%Y-%m-%d_%H%M%S')
    report_path = LOG_DIR / f'recovery_{timestamp}.json'
    
    report = {
        'generated_at': datetime.now(ARIZONA_TZ).isoformat(),
        'dry_run': dry_run,
        'summary': results,
        'total_processed': len(appointments)
    }
    
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)
    
    logger.info(f"Report saved: {report_path}")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Recovery workflow for missed appointments')
    parser.add_argument('--dry-run', action='store_true', help='Preview without making changes')
    
    args = parser.parse_args()
    
    try:
        run_recovery(dry_run=args.dry_run)
    except Exception as e:
        logger.exception(f"Recovery workflow failed: {e}")


if __name__ == '__main__':
    main()
