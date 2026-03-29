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
    const calibrationStatusEl = document.getElementById('calibrationStatus');
    if (calibrationStatusEl) {
      const selectedOption = selectEl.options[selectEl.selectedIndex];
      calibrationStatusEl.textContent = selectedOption ? `Cal: ${selectedOption.textContent}` : 'Cal: --';
    }
  } catch (error) {
    console.log('Error fetching calibrations:', error);
  }
}

function updateSpeedOffsetStatus(mode, offsetValue) {
  const offsetTypeEl = document.getElementById('speedOffsetType');
  const currentOffsetEl = document.getElementById('currentSpeedOffset');

  if (offsetTypeEl) {
    offsetTypeEl.textContent = mode || '--';
  }

  if (currentOffsetEl) {
    if (offsetValue === undefined || offsetValue === null || Number.isNaN(Number(offsetValue))) {
      currentOffsetEl.textContent = '--';
    } else {
      const n = Number(offsetValue);
      currentOffsetEl.textContent = (n > 0 ? '+' : '') + n;
    }
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
  const configInputs = [
    'hasNeedleSweep',
    'sweepSpeed',
    'stepRPM',
    'stepSpeed',
    'coilType',
    'motorCalibration',
    'maxSpeed',
    'maxFreqHall',
    'useGlobalSpeedOffset',
    'speedOffsetPositive',
    'speedOffset'
  ];
  configInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => {
        const value = el.type === 'checkbox' ? el.checked : el.value;
        pushControl(id, value);

        if (id === 'motorCalibration') {
          const selectedOption = el.options[el.selectedIndex];
          const calibrationStatusEl = document.getElementById('calibrationStatus');
          if (selectedOption && calibrationStatusEl) {
            calibrationStatusEl.textContent = `Cal: ${selectedOption.textContent}`;
          }
        }
      });

      if (el.type === 'range') {
        el.addEventListener('input', () => {
          const displayEl = document.getElementById(id + '-display');
          if (displayEl) {
            displayEl.textContent = el.value;
          }
          pushControl(id, el.value);
        });
      }
    }
  });

  const calibrationInputs = ['useSpeedOffsetCurve', 'curveOffset0', 'curveOffset1', 'curveOffset2', 'curveOffset3', 'curveOffset4'];
  calibrationInputs.forEach(id => {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }

    el.addEventListener('change', () => {
      const value = el.type === 'checkbox' ? el.checked : el.value;
      pushControl(id, value);
    });

    if (el.type === 'range') {
      el.addEventListener('input', () => {
        const displayEl = document.getElementById(id + '-display');
        if (displayEl) {
          displayEl.textContent = el.value;
        }
        pushControl(id, el.value);
      });
    }
  });

  const filterInputs = ['averageFilterHall', 'averageFilterRPM'];
  filterInputs.forEach(id => {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }

    el.addEventListener('change', () => {
      pushControl(id, el.value);
    });

    el.addEventListener('input', () => {
      const displayEl = document.getElementById(id + '-display');
      if (displayEl) {
        displayEl.textContent = el.value;
      }
      pushControl(id, el.value);
    });
  });

  // Advanced test controls
  const advancedInputs = [
    'testRPM', 'tempRPM', 'testSpeedo', 'tempSpeed', 'maxRPM', 'clusterRPMLimit', 'testCal',
    'broadcastSpeedEnabled', 'broadcastSpeedID', 'broadcastSpeedDLC',
    'broadcastSpeedLowByte', 'broadcastSpeedHighByte', 'broadcastSpeedLittleEndian',
    'broadcastSpeedScale', 'broadcastSpeedOffset',
    'broadcastSpeedData0', 'broadcastSpeedData1', 'broadcastSpeedData2', 'broadcastSpeedData3',
    'broadcastSpeedData4', 'broadcastSpeedData5', 'broadcastSpeedData6', 'broadcastSpeedData7'
  ];
  advancedInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => {
        let value;
        if (el.type === 'checkbox') {
          value = el.checked;
        } else if (id === 'broadcastSpeedID') {
          value = el.value.trim();
        } else if (id === 'broadcastSpeedLittleEndian') {
          value = el.value === 'true';
        } else {
          value = el.type === 'number' || el.type === 'range' ? Number(el.value) : el.value;
        }

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

      // For sliders, also update live display and send immediately on input
      if (el.type === 'range') {
        el.addEventListener('input', () => {
          const displayId = id + '-display';
          const displayEl = document.getElementById(displayId);
          if (displayEl) {
            displayEl.textContent = el.value;
          }
          if (id === 'tempRPM') {
            const testRPMCheckbox = document.getElementById('testRPM');
            pushTestRPM(testRPMCheckbox.checked, parseInt(el.value || 0));
          } else if (id === 'tempSpeed') {
            const testSpeedCheckbox = document.getElementById('testSpeedo');
            pushTestSpeed(testSpeedCheckbox.checked, parseInt(el.value || 0));
          } else {
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
    document.getElementById('broadcastSpeedEnabled').checked = data.broadcastSpeedEnabled || false;
    document.getElementById('broadcastSpeedID').value = (data.broadcastSpeedID || 0).toString(16).toUpperCase();
    document.getElementById('broadcastSpeedDLC').value = data.broadcastSpeedDLC ?? 8;
    document.getElementById('broadcastSpeedLowByte').value = data.broadcastSpeedLowByte ?? 2;
    document.getElementById('broadcastSpeedHighByte').value = data.broadcastSpeedHighByte ?? 3;
    document.getElementById('broadcastSpeedLittleEndian').value = (data.broadcastSpeedLittleEndian !== false) ? 'true' : 'false';
    document.getElementById('broadcastSpeedScale').value = (data.broadcastSpeedScale ?? 0.781).toFixed(3);
    document.getElementById('broadcastSpeedOffset').value = data.broadcastSpeedOffset ?? 0;
    for (let i = 0; i < 8; i++) {
      const dataEl = document.getElementById(`broadcastSpeedData${i}`);
      if (dataEl) dataEl.value = data[`broadcastSpeedData${i}`] ?? 0;
    }
    document.getElementById('sweepSpeed').value = data.sweepSpeed || 0;
    document.getElementById('stepRPM').value = data.stepRPM || 100;
    document.getElementById('stepSpeed').value = data.stepSpeed || 100;
    document.getElementById('coilType').checked = data.coilType || false;
    currentCalibrationId = parseInt(data.motorCalibration || currentCalibrationId || 1, 10) || 1;
    document.getElementById('motorCalibration').value = String(currentCalibrationId);
    document.getElementById('maxSpeed').value = data.maxSpeed || 200;
    document.getElementById('maxSpeed-display').textContent = data.maxSpeed || 200;
    document.getElementById('maxFreqHall').value = data.maxFreqHall || 200;
    document.getElementById('maxFreqHall-display').textContent = data.maxFreqHall || 200;
    document.getElementById('useGlobalSpeedOffset').checked = data.useGlobalSpeedOffset !== false;
    document.getElementById('speedOffsetPositive').checked = data.speedOffsetPositive !== false;
    document.getElementById('speedOffset').value = data.speedOffset || 0;
    document.getElementById('speedOffset-display').textContent = data.speedOffset || 0;
    document.getElementById('useSpeedOffsetCurve').checked = data.useSpeedOffsetCurve || false;

    const curveOffsets = Array.isArray(data.speedOffsetCurveOffsets) ? data.speedOffsetCurveOffsets : [0, 0, 0, 0, 0];
    for (let i = 0; i < 5; i++) {
      const offsetId = 'curveOffset' + i;
      const offsetVal = curveOffsets[i] ?? 0;
      const offsetEl = document.getElementById(offsetId);
      const displayEl = document.getElementById(offsetId + '-display');
      if (offsetEl) {
        offsetEl.value = offsetVal;
      }
      if (displayEl) {
        displayEl.textContent = offsetVal;
      }
    }

    document.getElementById('averageFilterHall').value = data.averageFilterHall || data.averageFilter || 6;
    document.getElementById('averageFilterHall-display').textContent = data.averageFilterHall || data.averageFilter || 6;
    document.getElementById('averageFilterRPM').value = data.averageFilterRPM || data.averageFilter || 6;
    document.getElementById('averageFilterRPM-display').textContent = data.averageFilterRPM || data.averageFilter || 6;
    updateSpeedOffsetStatus(data.speedOffsetType, data.currentSpeedOffset);

    const calibrationStatusEl = document.getElementById('calibrationStatus');
    if (calibrationStatusEl) {
      const selectedOption = document.getElementById('motorCalibration').selectedOptions[0];
      calibrationStatusEl.textContent = selectedOption ? `Cal: ${selectedOption.textContent}` : 'Cal: --';
    }

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
      } else if (data.gpsUnavailable) {
        document.getElementById('liveGPSStatus').textContent = 'Unavailable';
      } else {
        document.getElementById('liveGPSStatus').textContent = 'Not Connected';
      }
    }

    updateSpeedOffsetStatus(data.speedOffsetType, data.currentSpeedOffset);

    // System status (read-only, not settings)
    document.getElementById('canStatus').textContent = data.hasCAN ? 'CAN: Healthy' : 'CAN: Not Healthy';
    document.getElementById('broadcastStatus').textContent = data.broadcastSpeedEnabled ? 'Broadcast: Active' : 'Broadcast: Off';
    document.getElementById('canPresent').textContent = data.hasCAN ? 'Healthy' : 'Not Healthy';

    if (document.getElementById('liveBroadcastSpeedValue')) {
      const suffix = data.broadcastSpeedEnabled ? '' : ' (disabled)';
      document.getElementById('liveBroadcastSpeedValue').textContent = `${data.broadcastSpeedValue || 0}${suffix}`;
    }
    
    // GPS status in dashboard
    if (document.getElementById('gpsPresent')) {
      if (data.hasGPS) {
        document.getElementById('gpsPresent').textContent = `Connected (${data.gpsSatellites} sat)`;
      } else if (data.gpsUnavailable) {
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
