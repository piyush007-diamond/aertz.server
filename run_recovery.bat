@echo off
REM Recovery Workflow - Windows Batch Launcher
REM Runs at 1:00 PM IST (12:30 AM Arizona Time)
REM 30 minutes after verification to process missed appointments

cd /d "C:\Users\Piyush\Downloads\agents\execution"

echo ============================================
echo Missed Appointment Recovery Workflow
echo Started: %DATE% %TIME%
echo ============================================

python recovery_workflow.py

echo ============================================
echo Completed: %DATE% %TIME%
echo ============================================
