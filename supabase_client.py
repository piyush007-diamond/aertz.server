"""
Supabase Client for Appointment Verification
=============================================
Handles all Supabase database operations for the patients table.

Usage:
    python supabase_client.py --test           # Test connection
    python supabase_client.py --today          # Get today's appointments
    python supabase_client.py --new-today      # Get rows created today
    python supabase_client.py --date 2026-01-08  # Get appointments for specific date
"""

import os
import sys
from datetime import datetime, date
from pathlib import Path
from dotenv import load_dotenv
from supabase import create_client, Client

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SUPABASE_URL = os.getenv('SUPABASE_URL')
SUPABASE_KEY = os.getenv('SUPABASE_ANON_KEY')
TABLE_NAME = os.getenv('APPOINTMENT_TABLE', 'patients')


def get_supabase_client() -> Client:
    """Create and return a Supabase client."""
    if not SUPABASE_URL or not SUPABASE_KEY:
        raise ValueError("Missing SUPABASE_URL or SUPABASE_ANON_KEY in .env")
    return create_client(SUPABASE_URL, SUPABASE_KEY)


def get_appointments_for_date(target_date: date) -> list[dict]:
    """
    Fetch all appointments for a specific date.
    
    Args:
        target_date: The date to fetch appointments for
        
    Returns:
        List of appointment dictionaries
    """
    client = get_supabase_client()
    date_str = target_date.strftime('%Y-%m-%d')
    
    response = client.table(TABLE_NAME)\
        .select('*')\
        .eq('appointment_date', date_str)\
        .execute()
    
    return response.data


def get_today_appointments() -> list[dict]:
    """Fetch all appointments scheduled for today."""
    return get_appointments_for_date(date.today())


def get_new_rows_today() -> list[dict]:
    """
    Fetch all rows created today (based on created_at timestamp).
    
    Returns:
        List of patient/appointment dictionaries created today
    """
    client = get_supabase_client()
    today_start = datetime.now().replace(hour=0, minute=0, second=0, microsecond=0)
    today_start_str = today_start.isoformat()
    
    response = client.table(TABLE_NAME)\
        .select('*')\
        .gte('created_at', today_start_str)\
        .execute()
    
    return response.data


def get_all_appointments() -> list[dict]:
    """Fetch all appointments from the database."""
    client = get_supabase_client()
    response = client.table(TABLE_NAME).select('*').execute()
    return response.data


def test_connection() -> bool:
    """Test the Supabase connection and print status."""
    try:
        client = get_supabase_client()
        # Try to fetch one row
        response = client.table(TABLE_NAME).select('id').limit(1).execute()
        print(f"✅ Supabase connection successful!")
        print(f"   URL: {SUPABASE_URL}")
        print(f"   Table: {TABLE_NAME}")
        print(f"   Sample data retrieved: {len(response.data)} row(s)")
        return True
    except Exception as e:
        print(f"❌ Supabase connection failed!")
        print(f"   Error: {e}")
        return False


def format_appointment(appt: dict) -> str:
    """Format an appointment for display."""
    name = appt.get('name', 'Unknown')
    appt_date = appt.get('appointment_date', 'N/A')
    appt_time = appt.get('appointment_time', 'N/A')
    description = appt.get('description', '')
    email = appt.get('email', 'N/A')
    
    return f"  [{appt.get('id', '?')}] {name} | {appt_date} @ {appt_time} | {email} | {description}"


def main():
    """CLI entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Supabase Appointment Client')
    parser.add_argument('--test', action='store_true', help='Test connection')
    parser.add_argument('--today', action='store_true', help='Get today\'s appointments')
    parser.add_argument('--new-today', action='store_true', help='Get rows created today')
    parser.add_argument('--date', type=str, help='Get appointments for date (YYYY-MM-DD)')
    parser.add_argument('--all', action='store_true', help='Get all appointments')
    
    args = parser.parse_args()
    
    if args.test:
        test_connection()
    elif args.today:
        appointments = get_today_appointments()
        print(f"\n📅 Appointments for today ({date.today()}):")
        print(f"   Found {len(appointments)} appointment(s)\n")
        for appt in appointments:
            print(format_appointment(appt))
    elif args.new_today:
        rows = get_new_rows_today()
        print(f"\n🆕 New rows created today:")
        print(f"   Found {len(rows)} row(s)\n")
        for row in rows:
            print(format_appointment(row))
    elif args.date:
        try:
            target_date = datetime.strptime(args.date, '%Y-%m-%d').date()
            appointments = get_appointments_for_date(target_date)
            print(f"\n📅 Appointments for {target_date}:")
            print(f"   Found {len(appointments)} appointment(s)\n")
            for appt in appointments:
                print(format_appointment(appt))
        except ValueError:
            print("❌ Invalid date format. Use YYYY-MM-DD")
            sys.exit(1)
    elif args.all:
        appointments = get_all_appointments()
        print(f"\n📋 All appointments in database:")
        print(f"   Found {len(appointments)} appointment(s)\n")
        for appt in appointments:
            print(format_appointment(appt))
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
