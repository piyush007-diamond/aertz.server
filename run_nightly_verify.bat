@echo off
REM Nightly Appointment Verification - Windows Batch Launcher
REM Runs at 12:00 AM Arizona Time (MST - UTC-7)
REM 
REM To set up Task Scheduler, run this in PowerShell as Administrator:
REM   schtasks /create /tn "AppointmentVerification" /tr "C:\Users\Piyush\Downloads\agents\execution\run_nightly_verify.bat" /sc daily /st 00:00 /tz "US Mountain Standard Time"

cd /d "C:\Users\Piyush\Downloads\agents\execution"

echo ============================================
echo Nightly Appointment Verification
echo Started: %DATE% %TIME%
echo ============================================

python scheduled_verify.py

echo ============================================
echo Completed: %DATE% %TIME%
echo ============================================
