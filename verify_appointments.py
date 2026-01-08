"""
Appointment Verification Script
================================
Main workflow script that verifies Supabase appointments against Google Calendar events.

This script:
1. Fetches new appointments created today from Supabase
2. Checks the appointment_date for each row
3. Fetches Google Calendar events for those dates
4. Verifies if each appointment exists in the calendar
5. Generates a report of matches and mismatches

Usage:
    python verify_appointments.py                    # Verify new rows created today
    python verify_appointments.py --date 2026-01-08  # Verify specific date's appointments
    python verify_appointments.py --all              # Verify all appointments
    python verify_appointments.py --output report.json  # Save report to file
"""

import os
import sys
import json
import csv
from datetime import datetime, date
from pathlib import Path
from collections import defaultdict

# Import our custom clients
from supabase_client import (
    get_new_rows_today,
    get_appointments_for_date,
    get_all_appointments,
    test_connection as test_supabase
)
from google_calendar_client import (
    get_events_for_date,
    find_matching_event,
    parse_event_time,
    test_connection as test_gcal
)


class VerificationResult:
    """Represents the result of verifying an appointment."""
    
    def __init__(self, appointment: dict, status: str, calendar_event: dict = None, notes: str = ""):
        self.appointment = appointment
        self.status = status  # 'matched', 'missing', 'error'
        self.calendar_event = calendar_event
        self.notes = notes
    
    def to_dict(self) -> dict:
        return {
            'appointment_id': self.appointment.get('id'),
            'patient_name': self.appointment.get('name'),
            'appointment_date': self.appointment.get('appointment_date'),
            'appointment_time': self.appointment.get('appointment_time'),
            'email': self.appointment.get('email'),
            'status': self.status,
            'calendar_event_title': self.calendar_event.get('summary') if self.calendar_event else None,
            'notes': self.notes
        }


def verify_appointment(appointment: dict, events_cache: dict = None) -> VerificationResult:
    """
    Verify a single appointment against Google Calendar.
    
    Args:
        appointment: Appointment dict from Supabase
        events_cache: Optional cache of events by date to avoid repeated API calls
    """
    appt_date_str = appointment.get('appointment_date')
    
    if not appt_date_str:
        return VerificationResult(
            appointment, 
            'error', 
            notes='No appointment_date found'
        )
    
    try:
        appt_date = datetime.strptime(appt_date_str, '%Y-%m-%d').date()
    except ValueError:
        return VerificationResult(
            appointment,
            'error',
            notes=f'Invalid date format: {appt_date_str}'
        )
    
    # Get events for this date (use cache if available)
    if events_cache is not None and appt_date_str in events_cache:
        events = events_cache[appt_date_str]
    else:
        events = get_events_for_date(appt_date)
        if events_cache is not None:
            events_cache[appt_date_str] = events
    
    # Try to find a matching event
    matching_event = find_matching_event(appointment, events)
    
    if matching_event:
        return VerificationResult(
            appointment,
            'matched',
            calendar_event=matching_event,
            notes=f"Matched with: {matching_event.get('summary', 'Unknown')}"
        )
    else:
        return VerificationResult(
            appointment,
            'missing',
            notes='No matching calendar event found'
        )


def verify_appointments(appointments: list[dict]) -> list[VerificationResult]:
    """
    Verify a list of appointments against Google Calendar.
    
    Args:
        appointments: List of appointment dicts from Supabase
        
    Returns:
        List of VerificationResult objects
    """
    results = []
    events_cache = {}  # Cache events by date to minimize API calls
    
    print(f"\n🔍 Verifying {len(appointments)} appointment(s)...\n")
    
    for i, appt in enumerate(appointments, 1):
        name = appt.get('name', 'Unknown')
        appt_date = appt.get('appointment_date', 'N/A')
        appt_time = appt.get('appointment_time', 'N/A')
        
        print(f"  [{i}/{len(appointments)}] Checking: {name} @ {appt_date} {appt_time}...", end=' ')
        
        result = verify_appointment(appt, events_cache)
        results.append(result)
        
        if result.status == 'matched':
            print("✅ FOUND")
        elif result.status == 'missing':
            print("❌ NOT FOUND")
        else:
            print(f"⚠️ ERROR: {result.notes}")
    
    return results


def generate_report(results: list[VerificationResult]) -> dict:
    """Generate a summary report from verification results."""
    matched = [r for r in results if r.status == 'matched']
    missing = [r for r in results if r.status == 'missing']
    errors = [r for r in results if r.status == 'error']
    
    report = {
        'generated_at': datetime.now().isoformat(),
        'summary': {
            'total': len(results),
            'matched': len(matched),
            'missing': len(missing),
            'errors': len(errors),
            'match_rate': f"{len(matched)/len(results)*100:.1f}%" if results else "N/A"
        },
        'matched_appointments': [r.to_dict() for r in matched],
        'missing_appointments': [r.to_dict() for r in missing],
        'errors': [r.to_dict() for r in errors]
    }
    
    return report


def print_report(report: dict):
    """Print a formatted report to console."""
    print("\n" + "="*60)
    print("📊 APPOINTMENT VERIFICATION REPORT")
    print("="*60)
    print(f"Generated: {report['generated_at']}")
    print()
    
    summary = report['summary']
    print(f"Total Appointments Checked: {summary['total']}")
    print(f"  ✅ Matched (in calendar):  {summary['matched']}")
    print(f"  ❌ Missing (not in calendar): {summary['missing']}")
    print(f"  ⚠️ Errors:                  {summary['errors']}")
    print(f"  📈 Match Rate:              {summary['match_rate']}")
    
    if report['missing_appointments']:
        print("\n" + "-"*60)
        print("❌ MISSING APPOINTMENTS (Not found in Google Calendar):")
        print("-"*60)
        for appt in report['missing_appointments']:
            print(f"  ID {appt['appointment_id']}: {appt['patient_name']}")
            print(f"     Date: {appt['appointment_date']} @ {appt['appointment_time']}")
            print(f"     Email: {appt['email']}")
            print()
    
    if report['errors']:
        print("\n" + "-"*60)
        print("⚠️ ERRORS:")
        print("-"*60)
        for appt in report['errors']:
            print(f"  ID {appt['appointment_id']}: {appt['notes']}")
    
    print("\n" + "="*60)


def save_report(report: dict, output_path: str):
    """Save report to file (JSON or CSV based on extension)."""
    path = Path(output_path)
    
    if path.suffix.lower() == '.json':
        with open(path, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\n💾 Report saved to: {path}")
    
    elif path.suffix.lower() == '.csv':
        # Flatten for CSV
        all_results = (
            report['matched_appointments'] + 
            report['missing_appointments'] + 
            report['errors']
        )
        
        if all_results:
            with open(path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=all_results[0].keys())
                writer.writeheader()
                writer.writerows(all_results)
            print(f"\n💾 Report saved to: {path}")
        else:
            print("\n⚠️ No results to save to CSV")
    
    else:
        print(f"\n⚠️ Unknown file format: {path.suffix}. Use .json or .csv")


def main():
    """CLI entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Verify Supabase appointments against Google Calendar',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python verify_appointments.py                    # Verify new rows created today
  python verify_appointments.py --date 2026-01-08  # Verify specific date
  python verify_appointments.py --all              # Verify all appointments
  python verify_appointments.py --output report.json  # Save to file
        """
    )
    parser.add_argument('--test', action='store_true', help='Test both connections')
    parser.add_argument('--date', type=str, help='Verify appointments for specific date (YYYY-MM-DD)')
    parser.add_argument('--all', action='store_true', help='Verify all appointments')
    parser.add_argument('--new-today', action='store_true', help='Verify new rows created today (default)')
    parser.add_argument('--output', type=str, help='Save report to file (.json or .csv)')
    
    args = parser.parse_args()
    
    # Test connections first
    if args.test:
        print("\n🔌 Testing connections...\n")
        supabase_ok = test_supabase()
        print()
        gcal_ok = test_gcal()
        
        if supabase_ok and gcal_ok:
            print("\n✅ All connections successful!")
        else:
            print("\n❌ Some connections failed. Please check configuration.")
        return
    
    # Determine which appointments to verify
    if args.date:
        try:
            target_date = datetime.strptime(args.date, '%Y-%m-%d').date()
            print(f"\n📅 Fetching appointments for {target_date}...")
            appointments = get_appointments_for_date(target_date)
        except ValueError:
            print("❌ Invalid date format. Use YYYY-MM-DD")
            sys.exit(1)
    elif args.all:
        print("\n📋 Fetching all appointments...")
        appointments = get_all_appointments()
    else:
        # Default: new rows created today
        print(f"\n🆕 Fetching new appointments created today ({date.today()})...")
        appointments = get_new_rows_today()
    
    if not appointments:
        print("\n⚠️ No appointments found to verify.")
        return
    
    print(f"   Found {len(appointments)} appointment(s)")
    
    # Verify appointments
    results = verify_appointments(appointments)
    
    # Generate and print report
    report = generate_report(results)
    print_report(report)
    
    # Save to file if requested
    if args.output:
        save_report(report, args.output)


if __name__ == '__main__':
    main()
