"""
Booking API Server - Flask backend for booking page
====================================================
Provides real-time calendar slot availability to the booking page.

Usage:
    python booking_api.py           # Start server on port 5000
    python booking_api.py --port 8080  # Custom port
"""

import os
import json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo
from flask import Flask, jsonify, request
from flask_cors import CORS

from dotenv import load_dotenv
from slot_checker import get_available_slots, is_slot_available
from calendar_booker import create_appointment

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

app = Flask(__name__)
CORS(app)  # Allow cross-origin requests

ARIZONA_TZ = ZoneInfo('America/Phoenix')


@app.route('/api/slots', methods=['GET'])
def get_slots():
    """
    Get available time slots for a given date.
    
    Query params:
        date: Date in YYYY-MM-DD format
        
    Returns:
        JSON with available_slots array
    """
    date_str = request.args.get('date')
    
    if not date_str:
        return jsonify({'error': 'date parameter required'}), 400
    
    try:
        # Validate date format
        datetime.strptime(date_str, '%Y-%m-%d')
    except ValueError:
        return jsonify({'error': 'Invalid date format. Use YYYY-MM-DD'}), 400
    
    try:
        slots = get_available_slots(date_str)
        return jsonify({
            'date': date_str,
            'timezone': 'America/Phoenix',
            'available_slots': slots,
            'slot_duration_minutes': 60
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/check-slot', methods=['GET'])
def check_slot():
    """
    Check if a specific slot is available.
    
    Query params:
        date: Date in YYYY-MM-DD format
        time: Time in HH:MM format
        
    Returns:
        JSON with available boolean
    """
    date_str = request.args.get('date')
    time_str = request.args.get('time')
    
    if not date_str or not time_str:
        return jsonify({'error': 'date and time parameters required'}), 400
    
    try:
        available = is_slot_available(date_str, time_str)
        return jsonify({
            'date': date_str,
            'time': time_str,
            'available': available
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/book', methods=['POST'])
def book_appointment():
    """
    Book an appointment.
    
    Request body:
        date: Date in YYYY-MM-DD format
        time: Time in HH:MM format
        name: Patient name
        email: Patient email
        phone: Patient phone
        notes: Optional notes
        
    Returns:
        JSON with success status and event details
    """
    data = request.get_json()
    
    required_fields = ['date', 'time', 'name', 'email', 'phone']
    for field in required_fields:
        if not data.get(field):
            return jsonify({'error': f'{field} is required'}), 400
    
    # Re-check slot availability before booking
    if not is_slot_available(data['date'], data['time']):
        return jsonify({
            'error': 'Slot no longer available',
            'available': False
        }), 409
    
    try:
        event = create_appointment(
            date_str=data['date'],
            time_str=data['time'],
            patient_name=data['name'],
            email=data['email'],
            phone=data['phone'],
            description=data.get('notes', '')
        )
        
        if event:
            return jsonify({
                'success': True,
                'event_id': event.get('id'),
                'message': 'Appointment booked successfully'
            })
        else:
            return jsonify({'error': 'Failed to create appointment'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    return jsonify({
        'status': 'ok',
        'timestamp': datetime.now(ARIZONA_TZ).isoformat()
    })


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Booking API Server')
    parser.add_argument('--port', type=int, default=5000, help='Port to run on')
    parser.add_argument('--host', type=str, default='0.0.0.0', help='Host to bind')
    
    args = parser.parse_args()
    
    print(f"🚀 Starting Booking API Server...")
    print(f"   http://localhost:{args.port}/api/slots?date=2026-01-09")
    print(f"   http://localhost:{args.port}/api/health")
    
    app.run(host=args.host, port=args.port, debug=True)


if __name__ == '__main__':
    main()
