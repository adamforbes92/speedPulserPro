document.addEventListener('DOMContentLoaded', initApp);

let settingsLoaded = false;
let currentCalibrationId = 1;

// Tab navigation
function initApp() {
  initNavigation();
  initControls();
  fetchCalibrations().then(fetchSettings);  // Load calibration list before settings
  fetchStatus();    // Initial status fetch
  setInterval(fetchStatus, 200);  // Continue fetching live data only
}

async function fetchCalibrations() {
  const selectEl = document.getElementById('motorCalibration');
  if (!selectEl) {
    return;
  }

  try {
    const response = await fetch('/api/calibrations');
    const data = await response.json();
    const calibrations = Array.isArray(data.calibrations) ? data.calibrations : [];

    selectEl.innerHTML = '';

    calibrations.forEach(item => {
      const id = String(item.id);
      const option = document.createElement('option');
      option.value = id;
      option.textContent = `${id}: ${item.name}`;
      selectEl.appendChild(option);
    });

    if (calibrations.length === 0) {
      const option = document.createElement('option');
      option.value = '1';
      option.textContent = '1: Default calibration';
      selectEl.appendChild(option);
    }

    currentCalibrationId = parseInt(data.currentCalibrationId || 1, 10) || 1;
    selectEl.value = String(currentCalibrationId);
  } catch (error) {
    console.log('Error fetching calibrations:', error);
  }
}

function initNavigation() {
  const tabs = document.querySelectorAll(".nav-tab");
  const pages = document.querySelectorAll(".page");

  tabs.forEach((tab) => {
    tab.addEventListener("click", () => {
      const page = tab.dataset.page;

      tabs.forEach((t) => t.classList.remove("active"));
      tab.classList.add("active");

      pages.forEach((p) => p.classList.remove("active"));
      document.getElementById(`${page}-page`).classList.add("active");
    });
  });
}

function initControls() {
  // Dashboard controls
  const testBtn = document.getElementById('testNeedleSweep');
  if (testBtn) {
    testBtn.addEventListener('click', () => pushAction('needleSweep'));
  }

  const otaUploadBtn = document.getElementById('otaUploadBtn');
  if (otaUploadBtn) {
    otaUploadBtn.addEventListener('click', uploadFirmware);
  }

  const resetMaxRPMBtn = document.getElementById('resetMaxRPM');
  if (resetMaxRPMBtn) {
    resetMaxRPMBtn.addEventListener('click', () => {
      const slider = document.getElementById('maxRPM');
      const display = document.getElementById('maxRPM-display');
      slider.value = 230;
      display.textContent = '230';
      pushControl('maxRPM', 230);
    });
  }

  const resetClusterRPMLimitBtn = document.getElementById('resetClusterRPMLimit');
  if (resetClusterRPMLimitBtn) {
    resetClusterRPMLimitBtn.addEventListener('click', () => {
      const slider = document.getElementById('clusterRPMLimit');
      const display = document.getElementById('clusterRPMLimit-display');
      slider.value = 7000;
      display.textContent = '7000';
      pushControl('clusterRPMLimit', 7000);
    });
  }

  const dutyUpBtn = document.getElementById('dutyUpBtn');
  if (dutyUpBtn) {
    dutyUpBtn.addEventListener('click', () => {
      const display = document.getElementById('tempDutyCycle-display');
      const currentDuty = parseInt(display.textContent || '0', 10);
      const newDuty = currentDuty >= 385 ? 0 : currentDuty + 1;
      display.textContent = String(newDuty);
      pushControl('tempDutyCycle', newDuty);
    });
  }

  const dutyDownBtn = document.getElementById('dutyDownBtn');
  if (dutyDownBtn) {
    dutyDownBtn.addEventListener('click', () => {
      const display = document.getElementById('tempDutyCycle-display');
      const currentDuty = parseInt(display.textContent || '0', 10);
      const newDuty = currentDuty <= 0 ? 385 : currentDuty - 1;
      display.textContent = String(newDuty);
      pushControl('tempDutyCycle', newDuty);
    });
  }

  // Configuration controls
  const configInputs = ['hasNeedleSweep', 'broadcastSpeed', 'sweepSpeed', 'stepRPM', 'stepSpeed', 'coilType', 'motorCalibration'];
  configInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => {
        const value = el.type === 'checkbox' ? el.checked : el.value;
        pushControl(id, value);
      });
    }
  });

  // Advanced test controls
  const advancedInputs = ['testRPM', 'tempRPM', 'testSpeedo', 'tempSpeed', 'maxRPM', 'clusterRPMLimit', 'testCal'];
  advancedInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => {
        const value = el.type === 'checkbox' ? el.checked : el.value;
        
        // Use separate API endpoints for RPM and Speed tests
        if (id === 'testRPM') {
          pushTestRPM(el.checked, parseInt(document.getElementById('tempRPM').value || 0));
        } else if (id === 'tempRPM') {
          const testRPMCheckbox = document.getElementById('testRPM');
          pushTestRPM(testRPMCheckbox.checked, parseInt(el.value || 0));
        } else if (id === 'testSpeedo') {
          pushTestSpeed(el.checked, parseInt(document.getElementById('tempSpeed').value || 0));
        } else if (id === 'tempSpeed') {
          const testSpeedCheckbox = document.getElementById('testSpeedo');
          pushTestSpeed(testSpeedCheckbox.checked, parseInt(el.value || 0));
        } else {
          pushControl(id, value);
        }
      });
      
      // For sliders, also update live display and send immediately on input (instant feedback)
      if (el.type === 'range') {
        el.addEventListener('input', () => {
          const displayId = id + '-display';
          const displayEl = document.getElementById(displayId);
          if (displayEl) {
            displayEl.textContent = el.value;
          }
          
          // Send instant updates for test sliders and other controls
          if (id === 'tempRPM') {
            const testRPMCheckbox = document.getElementById('testRPM');
            pushTestRPM(testRPMCheckbox.checked, parseInt(el.value || 0));
          } else if (id === 'tempSpeed') {
            const testSpeedCheckbox = document.getElementById('testSpeedo');
            pushTestSpeed(testSpeedCheckbox.checked, parseInt(el.value || 0));
          } else {
            // For other sliders (maxRPM, clusterRPMLimit, testCal), send via pushControl
            pushControl(id, el.value);
          }
        });
      }
    }
  });

  // Speed source dropdown
  const speedSourceEl = document.getElementById('speedSource');
  if (speedSourceEl) {
    speedSourceEl.addEventListener('change', () => {
      pushControl('speedType', speedSourceEl.value);
    });
  }

  const rpmSourceEl = document.getElementById('rpmSource');
  if (rpmSourceEl) {
    rpmSourceEl.addEventListener('change', () => {
      pushControl('rpmType', rpmSourceEl.value);
    });
  }
}

async function uploadFirmware() {
  const fileInput = document.getElementById('otaBinFile');
  const statusEl = document.getElementById('otaStatus');
  const uploadBtn = document.getElementById('otaUploadBtn');

  if (!fileInput || !fileInput.files || fileInput.files.length === 0) {
    if (statusEl) statusEl.textContent = 'Please select a .bin file first';
    return;
  }

  const file = fileInput.files[0];
  if (!file.name.toLowerCase().endsWith('.bin')) {
    if (statusEl) statusEl.textContent = 'Invalid file type. Please choose a .bin firmware';
    return;
  }

  const formData = new FormData();
  formData.append('firmware', file, file.name);

  try {
    if (statusEl) statusEl.textContent = 'Uploading... do not power off';
    if (uploadBtn) uploadBtn.disabled = true;

    const response = await fetch('/api/ota', {
      method: 'POST',
      body: formData
    });

    if (response.ok) {
      if (statusEl) statusEl.textContent = 'Upload complete. Device rebooting...';
    } else {
      if (statusEl) statusEl.textContent = 'Upload failed. Please try again';
    }
  } catch (error) {
    if (statusEl) statusEl.textContent = 'Upload failed. Check connection and retry';
  } finally {
    if (uploadBtn) uploadBtn.disabled = false;
  }
}

async function fetchSettings() {
  try {
    const response = await fetch('/api/settings');
    const data = await response.json();

    // Load all settings from API once
    document.getElementById('hasNeedleSweep').checked = data.hasNeedleSweep || false;
    document.getElementById('broadcastSpeed').checked = data.broadcastSpeed || false;
    document.getElementById('sweepSpeed').value = data.sweepSpeed || 0;
    document.getElementById('stepRPM').value = data.stepRPM || 100;
    document.getElementById('stepSpeed').value = data.stepSpeed || 100;
    document.getElementById('coilType').checked = data.coilType || false;
    currentCalibrationId = parseInt(data.motorCalibration || currentCalibrationId || 1, 10) || 1;
    document.getElementById('motorCalibration').value = String(currentCalibrationId);

    // Advanced controls
    document.getElementById('testRPM').checked = data.testRPM || false;
    document.getElementById('tempRPM').value = data.tempRPM || 0;
    document.getElementById('tempRPM-display').textContent = data.tempRPM || 0;
    document.getElementById('testSpeedo').checked = data.testSpeedo || false;
    document.getElementById('tempSpeed').value = data.tempSpeed || 0;
    document.getElementById('tempSpeed-display').textContent = data.tempSpeed || 0;
    document.getElementById('testCal').checked = data.testCal || false;
    document.getElementById('tempDutyCycle-display').textContent = data.tempDutyCycle || 0;
    document.getElementById('maxRPM').value = data.maxRPM || 230;
    document.getElementById('maxRPM-display').textContent = data.maxRPM || 230;
    document.getElementById('clusterRPMLimit').value = data.clusterRPMLimit || 7000;
    document.getElementById('clusterRPMLimit-display').textContent = data.clusterRPMLimit || 7000;

    // Speed type dropdown - map speedType to dropdown options
    let speedTypeValue = 'Hall';  // default
    if (data.speedType === 'ECU') speedTypeValue = 'ECU';
    else if (data.speedType === 'ABS') speedTypeValue = 'ABS';
    else if (data.speedType === 'DSG') speedTypeValue = 'DSG';
    else if (data.speedType === 'TP2.0-DSG' || data.speedType === 'TP/UDS DSG') speedTypeValue = 'TP2.0-DSG';
    else if (data.speedType === 'GPS') speedTypeValue = 'GPS';
    document.getElementById('speedSource').value = speedTypeValue;

    // RPM source dropdown
    document.getElementById('rpmSource').value = (data.rpmType === 'CAN') ? 'CAN' : 'Hall';

    // Update FW version
    const fwResponse = await fetch('/api/settings');
    const fwData = await fwResponse.json();
    document.getElementById('fwVersion').textContent = 'FW: ' + (fwData.FW_VERSION || '--');

    settingsLoaded = true;
  } catch (error) {
    console.log('Error fetching settings:', error);
  }
}

async function fetchStatus() {
  try {
    const response = await fetch('/api/status');
    const data = await response.json();

    // Update dashboard live data
    document.getElementById('speed').textContent = data.vehicleSpeed || '--';
    document.getElementById('rpm').textContent = data.vehicleRPM || '--';
    
    // Add test mode indicators
    const speedElement = document.getElementById('speed');
    if (data.testSpeedo) {
      speedElement.title = 'Test Mode: ' + (data.tempSpeed || '0') + ' km/h';
      speedElement.style.color = 'orange';
    } else {
      speedElement.title = '';
      speedElement.style.color = '';
    }
    
    const rpmElement = document.getElementById('rpm');
    if (data.testRPM) {
      rpmElement.title = 'Test Mode: ' + (data.tempRPM || '0') + ' RPM';
      rpmElement.style.color = 'orange';
    } else {
      rpmElement.title = '';
      rpmElement.style.color = '';
    }

    // Update advanced live data - all speed sources
    if (document.getElementById('liveRPM')) {
      document.getElementById('liveRPM').textContent = data.vehicleRPM || '--';
    }
    if (document.getElementById('liveHallRPM')) {
      document.getElementById('liveHallRPM').textContent = data.hallRPM || '--';
    }
    if (document.getElementById('liveCANRPM')) {
      document.getElementById('liveCANRPM').textContent = data.canRPM || '--';
    }
    if (document.getElementById('liveHallSpeed')) {
      document.getElementById('liveHallSpeed').textContent = data.hallSpeed || '--';
    }
    if (document.getElementById('liveECUSpeed')) {
      document.getElementById('liveECUSpeed').textContent = data.ecuSpeed || '--';
    }
    if (document.getElementById('liveABSSpeed')) {
      document.getElementById('liveABSSpeed').textContent = data.absSpeed || '--';
    }
    if (document.getElementById('liveDSGSpeed')) {
      document.getElementById('liveDSGSpeed').textContent = data.dsgSpeed || '--';
    }
    if (document.getElementById('liveUDSSpeed')) {
      document.getElementById('liveUDSSpeed').textContent = data.udsSpeed || '--';
    }
    if (document.getElementById('liveGPSSpeed')) {
      document.getElementById('liveGPSSpeed').textContent = data.gpsSpeed || '--';
    }
    if (document.getElementById('tempDutyCycle-display')) {
      document.getElementById('tempDutyCycle-display').textContent = data.tempDutyCycle || 0;
    }
    if (document.getElementById('liveGPSStatus')) {
      if (data.hasGPS) {
        document.getElementById('liveGPSStatus').textContent = `Connected, ${data.gpsSatellites} satellites`;
      } else if (data.gpsTaskSuspended) {
        document.getElementById('liveGPSStatus').textContent = 'Unavailable';
      } else {
        document.getElementById('liveGPSStatus').textContent = 'Not Connected';
      }
    }

    // System status (read-only, not settings)
    document.getElementById('canStatus').textContent = data.hasCAN ? 'CAN: Healthy' : 'CAN: Not Healthy';
    document.getElementById('broadcastStatus').textContent = data.broadcastSpeed ? 'Broadcast: Active' : 'Broadcast: Off';
    document.getElementById('canPresent').textContent = data.hasCAN ? 'Healthy' : 'Not Healthy';
    
    // GPS status in dashboard
    if (document.getElementById('gpsPresent')) {
      if (data.hasGPS) {
        document.getElementById('gpsPresent').textContent = `Connected (${data.gpsSatellites} sat)`;
      } else if (data.gpsTaskSuspended) {
        document.getElementById('gpsPresent').textContent = 'Unavailable';
      } else {
        document.getElementById('gpsPresent').textContent = 'Not Connected';
      }
    }

  } catch (error) {
    console.log('Error fetching status:', error);
  }
}

function pushControl(key, value) {
  fetch('/api/control', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ key, value })
  }).catch(e => console.log('Control error:', e));
}

function pushAction(action) {
  fetch('/api/action', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action })
  }).catch(e => console.log('Action error:', e));
}

function pushTestRPM(enabled, value) {
  fetch('/api/testRPM', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled, value })
  }).catch(e => console.log('Test RPM error:', e));
}

function pushTestSpeed(enabled, value) {
  fetch('/api/testSpeed', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled, value })
  }).catch(e => console.log('Test Speed error:', e));
}

function hex2bin(hex) {
  return ("00000000" + parseInt(hex, 16).toString(2)).substr(-8);
}

function showNotification(message, type = "success") {
  const notification = document.createElement("div");
  notification.textContent = message;
  notification.style.cssText = `
        position: fixed;
        top: 20px;
        left: 50%;
        transform: translateX(-50%);
        padding: 1rem 2rem;
        background: ${type === "error" ? "var(--danger)" : "var(--success)"};
        color: white;
        border-radius: 8px;
        z-index: 10000;
        font-weight: 600;
        box-shadow: 0 4px 12px rgba(0,0,0,0.3);
    `;

  document.body.appendChild(notification);

  setTimeout(() => {
    notification.style.transition = "opacity 0.3s";
    notification.style.opacity = "0";
    setTimeout(() => notification.remove(), 300);
  }, 3000);
}
