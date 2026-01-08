"""
Email Sender - Send Recovery Emails via Gmail SMTP
====================================================
Send apology emails to patients with booking links using Gmail SMTP.

Usage:
    python email_sender.py --test
    python email_sender.py --to "patient@email.com" --name "John Doe" --date "2026-01-09" --time "14:00"
"""

import os
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from dotenv import load_dotenv

# Load environment variables
env_path = Path(__file__).parent / '.env'
load_dotenv(env_path)

# Configuration
SMTP_EMAIL = os.getenv('SMTP_EMAIL', 'piyushhire007@gmail.com')
SMTP_APP_PASSWORD = os.getenv('SMTP_APP_PASSWORD', '')
SMTP_SERVER = 'smtp.gmail.com'
SMTP_PORT = 587

CLINIC_NAME = os.getenv('CLINIC_NAME', 'Our Clinic')
BOOKING_PAGE_URL = os.getenv('BOOKING_PAGE_URL', 'file:///C:/Users/Piyush/Downloads/agents/execution/booking_page/index.html')

ARIZONA_TZ = ZoneInfo('America/Phoenix')


def create_recovery_email(
    to_email: str,
    patient_name: str,
    original_date: str,
    original_time: str,
    booking_link: str = None
) -> MIMEMultipart:
    """
    Create an apology/recovery email message.
    """
    if not booking_link:
        booking_link = BOOKING_PAGE_URL
    
    subject = f"Action Required: Reschedule Your Appointment - {CLINIC_NAME}"
    
    # HTML email body
    html_body = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <style>
            body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
            .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
            .header {{ background: #dc3545; color: white; padding: 20px; text-align: center; border-radius: 8px 8px 0 0; }}
            .content {{ background: #f9f9f9; padding: 30px; border-radius: 0 0 8px 8px; }}
            .button {{ display: inline-block; background: #007bff; color: white; padding: 15px 30px; 
                       text-decoration: none; border-radius: 5px; margin: 20px 0; font-weight: bold; }}
            .button:hover {{ background: #0056b3; }}
            .details {{ background: white; padding: 15px; border-radius: 5px; margin: 15px 0; }}
            .footer {{ text-align: center; color: #666; font-size: 12px; margin-top: 20px; }}
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h2>⚠️ Appointment Scheduling Issue</h2>
            </div>
            <div class="content">
                <p>Dear <strong>{patient_name}</strong>,</p>
                
                <p>We sincerely apologize. Due to a technical issue, your appointment was not properly scheduled in our system.</p>
                
                <div class="details">
                    <strong>Original Appointment Details:</strong><br>
                    📅 Date: {original_date}<br>
                    🕐 Time: {original_time} (Arizona Time)
                </div>
                
                <p>Please use the link below to book a new appointment at your convenience:</p>
                
                <p style="text-align: center;">
                    <a href="{booking_link}" class="button">📅 Book New Appointment</a>
                </p>
                
                <p>We deeply apologize for any inconvenience this may have caused. If you have any questions, 
                please don't hesitate to contact us.</p>
                
                <p>Best regards,<br>
                <strong>{CLINIC_NAME}</strong></p>
            </div>
            <div class="footer">
                <p>This is an automated message from {CLINIC_NAME}'s scheduling system.</p>
            </div>
        </div>
    </body>
    </html>
    """
    
    # Plain text fallback
    text_body = f"""
    Dear {patient_name},

    We sincerely apologize. Due to a technical issue, your appointment was not properly scheduled in our system.

    Original Appointment Details:
    - Date: {original_date}
    - Time: {original_time} (Arizona Time)

    Please use the link below to book a new appointment:
    {booking_link}

    We deeply apologize for any inconvenience.

    Best regards,
    {CLINIC_NAME}
    """
    
    message = MIMEMultipart('alternative')
    message['To'] = to_email
    message['From'] = f"{CLINIC_NAME} <{SMTP_EMAIL}>"
    message['Subject'] = subject
    
    message.attach(MIMEText(text_body, 'plain'))
    message.attach(MIMEText(html_body, 'html'))
    
    return message


def send_email(message: MIMEMultipart) -> bool:
    """
    Send an email via Gmail SMTP.
    """
    if not SMTP_APP_PASSWORD:
        print("❌ SMTP_APP_PASSWORD not configured in .env")
        return False
    
    try:
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SMTP_EMAIL, SMTP_APP_PASSWORD)
            server.send_message(message)
        
        print(f"✅ Email sent successfully!")
        print(f"   To: {message['To']}")
        
        return True
    except smtplib.SMTPAuthenticationError as e:
        print(f"❌ SMTP Authentication failed: {e}")
        return False
    except Exception as e:
        print(f"❌ Failed to send email: {e}")
        return False


def send_recovery_email(
    to_email: str,
    patient_name: str,
    original_date: str,
    original_time: str,
    booking_link: str = None
) -> bool:
    """
    Send a recovery email to a patient.
    """
    if not to_email or to_email == 'NULL' or '@' not in to_email:
        print(f"⚠️ Invalid email address: {to_email}")
        return False
    
    message = create_recovery_email(
        to_email=to_email,
        patient_name=patient_name,
        original_date=original_date,
        original_time=original_time,
        booking_link=booking_link
    )
    
    return send_email(message)


def test_connection():
    """Test SMTP connection by sending a test email to yourself."""
    try:
        print(f"Testing SMTP connection...")
        print(f"   Server: {SMTP_SERVER}:{SMTP_PORT}")
        print(f"   Email: {SMTP_EMAIL}")
        
        with smtplib.SMTP(SMTP_SERVER, SMTP_PORT) as server:
            server.starttls()
            server.login(SMTP_EMAIL, SMTP_APP_PASSWORD)
        
        print(f"✅ SMTP connection successful!")
        print(f"   Ready to send emails from: {SMTP_EMAIL}")
        
        return True
    except smtplib.SMTPAuthenticationError as e:
        print(f"❌ SMTP Authentication failed!")
        print(f"   Error: {e}")
        print(f"   Check your App Password in .env")
        return False
    except Exception as e:
        print(f"❌ SMTP connection failed!")
        print(f"   Error: {e}")
        return False


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Send recovery emails via Gmail SMTP')
    parser.add_argument('--test', action='store_true', help='Test SMTP connection')
    parser.add_argument('--send-test', action='store_true', help='Send a test email to yourself')
    parser.add_argument('--to', type=str, help='Recipient email')
    parser.add_argument('--name', type=str, help='Patient name')
    parser.add_argument('--date', type=str, help='Original appointment date')
    parser.add_argument('--time', type=str, help='Original appointment time')
    
    args = parser.parse_args()
    
    if args.test:
        test_connection()
    elif args.send_test:
        # Send a test email to yourself
        success = send_recovery_email(
            to_email=SMTP_EMAIL,
            patient_name="Test Patient",
            original_date="2026-01-09",
            original_time="14:00"
        )
        if success:
            print(f"📧 Test email sent to {SMTP_EMAIL}")
    elif args.to and args.name and args.date and args.time:
        send_recovery_email(
            to_email=args.to,
            patient_name=args.name,
            original_date=args.date,
            original_time=args.time
        )
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
