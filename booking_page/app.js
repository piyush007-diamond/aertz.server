/**
 * Booking Page JavaScript - With Real-Time Calendar Sync
 * Connects to booking_api.py for live slot availability
 */

// Configuration
const CONFIG = {
    timezone: 'America/Phoenix',
    workingHours: { start: 9, end: 17 },  // 9 AM - 5 PM
    slotDuration: 60,  // 1 hour slots
    // API endpoint - update this when deploying
    apiUrl: 'https://aertz-server.onrender.com/api'
};

// State
let state = {
    currentMonth: new Date(),
    selectedDate: null,
    selectedTime: null,
    availableSlots: [],
    isLoading: false
};

// DOM Elements
const elements = {
    stepDate: document.getElementById('step-date'),
    stepTime: document.getElementById('step-time'),
    stepDetails: document.getElementById('step-details'),
    stepConfirm: document.getElementById('step-confirm'),
    loading: document.getElementById('loading'),
    
    calendarGrid: document.getElementById('calendar-grid'),
    currentMonth: document.getElementById('current-month'),
    prevMonth: document.getElementById('prev-month'),
    nextMonth: document.getElementById('next-month'),
    
    timeSlots: document.getElementById('time-slots'),
    noSlots: document.getElementById('no-slots'),
    selectedDateDisplay: document.getElementById('selected-date-display'),
    
    backToDate: document.getElementById('back-to-date'),
    backToTime: document.getElementById('back-to-time'),
    
    summaryDate: document.getElementById('summary-date'),
    summaryTime: document.getElementById('summary-time'),
    
    bookingForm: document.getElementById('booking-form'),
    submitBtn: document.getElementById('submit-btn'),
    
    confirmDate: document.getElementById('confirm-date'),
    confirmTime: document.getElementById('confirm-time'),
    confirmName: document.getElementById('confirm-name'),
    confirmEmail: document.getElementById('confirm-email'),
    bookAnother: document.getElementById('book-another'),
    
    timezoneSelect: document.getElementById('timezone-select')
};

// Initialize
document.addEventListener('DOMContentLoaded', init);

function init() {
    renderCalendar();
    setupEventListeners();
    checkApiConnection();
}

async function checkApiConnection() {
    try {
        const response = await fetch(`${CONFIG.apiUrl}/health`);
        if (response.ok) {
            console.log('✅ API connected - real-time slots enabled');
        }
    } catch (e) {
        console.log('⚠️ API not available - using fallback mode');
    }
}

function setupEventListeners() {
    elements.prevMonth.addEventListener('click', () => {
        state.currentMonth.setMonth(state.currentMonth.getMonth() - 1);
        renderCalendar();
    });
    
    elements.nextMonth.addEventListener('click', () => {
        state.currentMonth.setMonth(state.currentMonth.getMonth() + 1);
        renderCalendar();
    });
    
    elements.backToDate.addEventListener('click', () => showStep('date'));
    elements.backToTime.addEventListener('click', () => showStep('time'));
    
    elements.bookingForm.addEventListener('submit', handleSubmit);
    elements.bookAnother.addEventListener('click', resetBooking);
    
    elements.timezoneSelect.addEventListener('change', (e) => {
        CONFIG.timezone = e.target.value;
    });
}

function showStep(stepName) {
    document.querySelectorAll('.step').forEach(s => s.classList.remove('active'));
    elements.loading.style.display = 'none';
    
    switch(stepName) {
        case 'date':
            elements.stepDate.classList.add('active');
            break;
        case 'time':
            elements.stepTime.classList.add('active');
            break;
        case 'details':
            elements.stepDetails.classList.add('active');
            break;
        case 'confirm':
            elements.stepConfirm.classList.add('active');
            break;
    }
}

function showLoading() {
    document.querySelectorAll('.step').forEach(s => s.classList.remove('active'));
    elements.loading.style.display = 'block';
}

// Calendar Functions
function renderCalendar() {
    const year = state.currentMonth.getFullYear();
    const month = state.currentMonth.getMonth();
    
    const monthNames = ['January', 'February', 'March', 'April', 'May', 'June',
                        'July', 'August', 'September', 'October', 'November', 'December'];
    elements.currentMonth.textContent = `${monthNames[month]} ${year}`;
    
    const firstDay = new Date(year, month, 1).getDay();
    const daysInMonth = new Date(year, month + 1, 0).getDate();
    const today = new Date();
    today.setHours(0, 0, 0, 0);
    
    const startOffset = firstDay === 0 ? 6 : firstDay - 1;
    
    elements.calendarGrid.innerHTML = '';
    
    for (let i = 0; i < startOffset; i++) {
        const emptyDay = document.createElement('div');
        emptyDay.className = 'calendar-day empty';
        elements.calendarGrid.appendChild(emptyDay);
    }
    
    for (let day = 1; day <= daysInMonth; day++) {
        const dateObj = new Date(year, month, day);
        const dayEl = document.createElement('div');
        dayEl.className = 'calendar-day';
        dayEl.textContent = day;
        
        if (dateObj < today) {
            dayEl.classList.add('disabled');
        } else {
            if (dateObj.getTime() === today.getTime()) {
                dayEl.classList.add('today');
            }
            
            if (state.selectedDate && 
                dateObj.getTime() === state.selectedDate.getTime()) {
                dayEl.classList.add('selected');
            }
            
            dayEl.addEventListener('click', () => selectDate(dateObj));
        }
        
        elements.calendarGrid.appendChild(dayEl);
    }
}

function selectDate(date) {
    state.selectedDate = date;
    renderCalendar();
    
    const options = { weekday: 'long', year: 'numeric', month: 'long', day: 'numeric' };
    elements.selectedDateDisplay.textContent = date.toLocaleDateString('en-US', options);
    
    loadTimeSlots(date);
}

// Time Slots Functions - Now with real API calls
async function loadTimeSlots(date) {
    showLoading();
    state.isLoading = true;
    
    const dateStr = formatDateISO(date);
    
    try {
        // Try to fetch from API first
        const response = await fetch(`${CONFIG.apiUrl}/slots?date=${dateStr}`);
        
        if (response.ok) {
            const data = await response.json();
            state.availableSlots = data.available_slots || [];
            console.log(`✅ Fetched ${state.availableSlots.length} slots from API`);
        } else {
            throw new Error('API error');
        }
    } catch (error) {
        console.log('⚠️ Using fallback slots (API unavailable)');
        // Fallback: show all working hours
        state.availableSlots = [];
        for (let hour = CONFIG.workingHours.start; hour < CONFIG.workingHours.end; hour++) {
            state.availableSlots.push(`${hour.toString().padStart(2, '0')}:00`);
        }
    }
    
    state.isLoading = false;
    renderTimeSlots();
    showStep('time');
}

function renderTimeSlots() {
    elements.timeSlots.innerHTML = '';
    
    if (state.availableSlots.length === 0) {
        elements.noSlots.style.display = 'block';
        elements.timeSlots.style.display = 'none';
        return;
    }
    
    elements.noSlots.style.display = 'none';
    elements.timeSlots.style.display = 'flex';
    
    state.availableSlots.forEach(slot => {
        const slotEl = document.createElement('div');
        slotEl.className = 'time-slot';
        slotEl.textContent = formatTime12Hour(slot);
        
        if (state.selectedTime === slot) {
            slotEl.classList.add('selected');
        }
        
        slotEl.addEventListener('click', () => selectTime(slot));
        elements.timeSlots.appendChild(slotEl);
    });
}

function selectTime(time) {
    state.selectedTime = time;
    
    const dateOptions = { year: 'numeric', month: 'long', day: 'numeric' };
    elements.summaryDate.textContent = state.selectedDate.toLocaleDateString('en-US', dateOptions);
    elements.summaryTime.textContent = formatTime12Hour(time);
    
    showStep('details');
}

// Form Submission - With real-time slot verification
async function handleSubmit(e) {
    e.preventDefault();
    
    const formData = {
        name: document.getElementById('name').value.trim(),
        email: document.getElementById('email').value.trim(),
        phone: document.getElementById('phone').value.trim(),
        notes: document.getElementById('notes').value.trim(),
        date: formatDateISO(state.selectedDate),
        time: state.selectedTime,
        timezone: CONFIG.timezone
    };
    
    // Validate
    if (!formData.name || !formData.email || !formData.phone) {
        alert('Please fill in all required fields');
        return;
    }
    
    showLoading();
    elements.submitBtn.disabled = true;
    
    try {
        // First, re-verify slot is still available
        const checkResponse = await fetch(`${CONFIG.apiUrl}/check-slot?date=${formData.date}&time=${formData.time}`);
        
        if (checkResponse.ok) {
            const checkData = await checkResponse.json();
            
            if (!checkData.available) {
                alert('Sorry, this slot was just booked by someone else. Please select a different time.');
                await loadTimeSlots(state.selectedDate);
                showStep('time');
                elements.submitBtn.disabled = false;
                return;
            }
        }
        
        // Book the appointment
        const response = await fetch(`${CONFIG.apiUrl}/book`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(formData)
        });
        
        if (response.ok) {
            const result = await response.json();
            if (result.success) {
                showConfirmation(formData);
            } else {
                throw new Error(result.error || 'Booking failed');
            }
        } else {
            const errorData = await response.json();
            if (errorData.error === 'Slot no longer available') {
                alert('Sorry, this slot was just booked. Please select a different time.');
                await loadTimeSlots(state.selectedDate);
                showStep('time');
            } else {
                throw new Error(errorData.error || 'Booking failed');
            }
        }
    } catch (error) {
        console.error('Booking error:', error);
        
        // Fallback for local file mode
        if (window.location.protocol === 'file:') {
            // Simulate success in local mode
            await new Promise(resolve => setTimeout(resolve, 1000));
            showConfirmation(formData);
        } else {
            alert('Sorry, there was an error. Please try again or contact us directly.');
            showStep('details');
        }
    }
    
    elements.submitBtn.disabled = false;
}

function showConfirmation(data) {
    const dateOptions = { year: 'numeric', month: 'long', day: 'numeric' };
    elements.confirmDate.textContent = state.selectedDate.toLocaleDateString('en-US', dateOptions);
    elements.confirmTime.textContent = formatTime12Hour(state.selectedTime);
    elements.confirmName.textContent = data.name;
    elements.confirmEmail.textContent = data.email;
    
    showStep('confirm');
}

function resetBooking() {
    state.selectedDate = null;
    state.selectedTime = null;
    state.availableSlots = [];
    
    document.getElementById('name').value = '';
    document.getElementById('email').value = '';
    document.getElementById('phone').value = '';
    document.getElementById('notes').value = '';
    
    renderCalendar();
    showStep('date');
}

// Utility Functions
function formatDateISO(date) {
    const year = date.getFullYear();
    const month = (date.getMonth() + 1).toString().padStart(2, '0');
    const day = date.getDate().toString().padStart(2, '0');
    return `${year}-${month}-${day}`;
}

function formatTime12Hour(time24) {
    const [hours, minutes] = time24.split(':');
    const hour = parseInt(hours);
    const ampm = hour >= 12 ? 'PM' : 'AM';
    const hour12 = hour % 12 || 12;
    return `${hour12}:${minutes || '00'} ${ampm}`;
}
