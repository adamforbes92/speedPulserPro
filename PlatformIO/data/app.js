document.addEventListener('DOMContentLoaded', initApp);

let settingsLoaded = false;
let currentCalibrationId = 1;

// Tab navigation
function initApp() {
  initNavigation();
  initControls();
  initOta();
  initCalBuilder();
  fetchCalibrations().then(fetchSettings).then(refreshCalState).then(fetchCalCurve);  // list, settings, cal builder state, then the curve
  fetchStatus();    // Initial status fetch
  setInterval(fetchStatus, 200);  // Continue fetching live data only
  window.addEventListener('resize', scheduleCalDraw);
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

// Show the global offset slider OR the 5-point curve, never both.
function applyOffsetCurveVisibility(enabled) {
  const globalSec = document.getElementById('globalOffsetSection');
  const curveSec = document.getElementById('offsetCurveSection');
  if (globalSec) globalSec.style.display = enabled ? 'none' : '';
  if (curveSec) curveSec.style.display = enabled ? '' : 'none';
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

      // The curve canvas can't measure itself while its tab is hidden, so
      // redraw once the Dashboard page becomes visible.
      if (page === 'dashboard') scheduleCalDraw();
    });
  });
}

function initControls() {
    // GPS Rate controls
    const gpsRateSelect = document.getElementById('gpsRateSelect');
    const setGpsRateBtn = document.getElementById('setGpsRateBtn');
    const gpsRateResponse = document.getElementById('gpsRateResponse');
    // Always populate dropdown for robustness
    if (gpsRateSelect) {
      const validRates = [1, 5, 10, 16];
      gpsRateSelect.innerHTML = '';
      validRates.forEach(rate => {
        const opt = document.createElement('option');
        opt.value = rate;
        opt.textContent = rate + ' Hz';
        gpsRateSelect.appendChild(opt);
      });
    }
    if (setGpsRateBtn && gpsRateSelect) {
      setGpsRateBtn.addEventListener('click', async () => {
        const rate = parseInt(gpsRateSelect.value, 10);
        gpsRateResponse.textContent = 'Sending...';
        window.gpsRateUserMessageUntil = Date.now() + 4000;
        try {
          const resp = await fetch('/api/gpsRate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ rate })
          });
          const data = await resp.json();
          gpsRateResponse.textContent = data.message || (data.success ? 'Success' : 'Failed');
          gpsRateResponse.style.color = data.success ? '#007a3d' : '#b00020';
          window.gpsRateUserMessageUntil = Date.now() + 4000;
        } catch (e) {
          gpsRateResponse.textContent = 'Error sending command.';
          gpsRateResponse.style.color = '#b00020';
          window.gpsRateUserMessageUntil = Date.now() + 4000;
        }
      });
    }
  // Dashboard controls
  const testBtn = document.getElementById('testNeedleSweep');
  if (testBtn) {
    testBtn.addEventListener('click', () => pushAction('needleSweep'));
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
    'stepSpeed',
    'stepRPM',
    'coilType',
    'convertToMPH',
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
        const ctrlPromise = pushControl(id, value);

        if (id === 'motorCalibration') {
          const selectedOption = el.options[el.selectedIndex];
          const calibrationStatusEl = document.getElementById('calibrationStatus');
          if (selectedOption && calibrationStatusEl) {
            calibrationStatusEl.textContent = `Cal: ${selectedOption.textContent}`;
          }
          // Refresh the curve only after the device has applied the new cal.
          if (ctrlPromise && ctrlPromise.then) ctrlPromise.then(fetchCalCurve); else fetchCalCurve();
        }

        if (id === 'convertToMPH') {
          applyMphState(el.checked);
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

  const resetMaxFreqHallBtn = document.getElementById('resetMaxFreqHall');
  const maxFreqHallEl = document.getElementById('maxFreqHall');
  if (resetMaxFreqHallBtn && maxFreqHallEl) {
    resetMaxFreqHallBtn.addEventListener('click', () => {
      maxFreqHallEl.value = 200;
      const displayEl = document.getElementById('maxFreqHall-display');
      if (displayEl) displayEl.textContent = '200';
      pushControl('maxFreqHall', 200);
    });
  }

  // CAN Analyzer - SavvyCAN: WiFi and Serial are mutually exclusive
  const analyzerModeEl = document.getElementById('analyzerMode');
  const analyzerSerialEl = document.getElementById('analyzerSerial');
  if (analyzerModeEl && analyzerSerialEl) {
    analyzerModeEl.addEventListener('change', () => {
      if (analyzerModeEl.checked) {
        analyzerSerialEl.checked = false;
        pushControl('analyzerSerial', false);
      }
      pushControl('analyzerMode', analyzerModeEl.checked);
    });
    analyzerSerialEl.addEventListener('change', () => {
      if (analyzerSerialEl.checked) {
        analyzerModeEl.checked = false;
        pushControl('analyzerMode', false);
      }
      pushControl('analyzerSerial', analyzerSerialEl.checked);
    });
  }

  const calibrationInputs = ['useSpeedOffsetCurve', 'curveOffset0', 'curveOffset1', 'curveOffset2', 'curveOffset3', 'curveOffset4'];
  calibrationInputs.forEach(id => {
    const el = document.getElementById(id);
    if (!el) {
      return;
    }

    el.addEventListener('change', () => {
      const value = el.type === 'checkbox' ? el.checked : el.value;
      pushControl(id, value);
      if (id === 'useSpeedOffsetCurve') {
        applyOffsetCurveVisibility(el.checked);
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
    'broadcastSpeedData4', 'broadcastSpeedData5', 'broadcastSpeedData6', 'broadcastSpeedData7',
    'aftermarketSpeedID', 'aftermarketSpeedLowByte', 'aftermarketSpeedHighByte',
    'aftermarketSpeedLittleEndian', 'aftermarketSpeedScale', 'aftermarketSpeedOffset'
  ];
  advancedInputs.forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => {
        let value;
        if (el.type === 'checkbox') {
          value = el.checked;
        } else if (id === 'broadcastSpeedID' || id === 'aftermarketSpeedID') {
          value = el.value.trim();
        } else if (id === 'broadcastSpeedLittleEndian' || id === 'aftermarketSpeedLittleEndian') {
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
  function updateCustomCANVisibility() {
    const card = document.getElementById('customCANInputCard');
    if (card && speedSourceEl) {
      card.style.display = speedSourceEl.value === 'Custom CAN' ? '' : 'none';
    }
  }
  if (speedSourceEl) {
    speedSourceEl.addEventListener('change', () => {
      pushControl('speedType', speedSourceEl.value);
      updateCustomCANVisibility();
    });
  }

  const rpmSourceEl = document.getElementById('rpmSource');
  if (rpmSourceEl) {
    rpmSourceEl.addEventListener('change', () => {
      pushControl('rpmType', rpmSourceEl.value);
    });
  }

  // Closed-loop feedback (PID) controls
  const feedbackEnableEl = document.getElementById('feedbackEnable');
  if (feedbackEnableEl) {
    feedbackEnableEl.addEventListener('change', () => {
      pushControl('feedbackEnable', feedbackEnableEl.checked);
    });
  }

  const reverseDirectionEl = document.getElementById('reverseDirection');
  if (reverseDirectionEl) {
    reverseDirectionEl.addEventListener('change', () => {
      pushControl('reverseDirection', reverseDirectionEl.checked);
    });
  }

  const feedbackMinSpeedEl = document.getElementById('feedbackMinSpeed');
  if (feedbackMinSpeedEl) {
    feedbackMinSpeedEl.addEventListener('input', () => {
      const displayEl = document.getElementById('feedbackMinSpeed-display');
      if (displayEl) displayEl.textContent = feedbackMinSpeedEl.value;
      pushControl('feedbackMinSpeed', feedbackMinSpeedEl.value);
    });
  }

  ['pidKp', 'pidKi', 'pidKd'].forEach(id => {
    const el = document.getElementById(id);
    if (el) {
      el.addEventListener('change', () => pushControl(id, el.value));
    }
  });

  const feedbackDeadbandEl = document.getElementById('feedbackDeadband');
  if (feedbackDeadbandEl) {
    feedbackDeadbandEl.addEventListener('input', () => {
      const displayEl = document.getElementById('feedbackDeadband-display');
      if (displayEl) displayEl.textContent = feedbackDeadbandEl.value;
      pushControl('feedbackDeadband', feedbackDeadbandEl.value);
    });
  }

  const pidDefaults = { pidKp: 0.15, pidKi: 1.3, pidKd: 0, feedbackDeadband: 1.5, feedbackMinSpeed: 40 };
  const resetPidBtn = document.getElementById('resetPidDefaults');
  if (resetPidBtn) {
    resetPidBtn.addEventListener('click', () => {
      Object.entries(pidDefaults).forEach(([id, val]) => {
        const el = document.getElementById(id);
        if (el) el.value = val;
        const displayEl = document.getElementById(id + '-display');
        if (displayEl) displayEl.textContent = val;
        pushControl(id, val);
      });
    });
  }

  initCollapsibleCards();
}

// Turn every settings page into a scannable accordion: each card header
// collapses/expands its body, and ALL cards start collapsed for a clean UI.
// The Dashboard is left open so live data + the curve are always visible.
function initCollapsibleCards() {
  // OTA is intentionally excluded — its card stays open (matches SpeedPulser).
  const pages = ['configuration', 'advanced', 'calibration', 'diag'];
  pages.forEach(pageId => {
    document.querySelectorAll(`#${pageId}-page .card`).forEach(card => {
      const header = card.querySelector('h2');
      if (!header) return;
      card.classList.add('collapsible', 'collapsed');
      header.addEventListener('click', () => card.classList.toggle('collapsed'));
    });
  });
}

// ===== OTA UPDATE (dropdown + drag/drop) =====
function initOta() {
  const dropZone     = document.getElementById('otaDropZone');
  const fileInput    = document.getElementById('otaFile');
  const fileNameEl   = document.getElementById('otaFileName');
  const uploadBtn    = document.getElementById('otaUploadBtn');
  const progressWrap = document.getElementById('otaProgressWrap');
  const progressBar  = document.getElementById('otaProgressBar');
  const progressLbl  = document.getElementById('otaProgressLabel');
  const statusEl     = document.getElementById('otaStatus');
  const chooseBtn    = document.getElementById('otaChooseBtn');
  const typeSelect   = document.getElementById('otaType');

  if (!dropZone) return;

  // Populate the Firmware Info card from the shared /api/version endpoint.
  fetch('/api/version')
    .then((r) => r.json())
    .then((info) => {
      const set = (id, val) => {
        const el = document.getElementById(id);
        if (el) el.textContent = val || '--';
      };
      set('otaFwVersion', info.version);
      set('otaHardware', info.hardware);
      set('otaBoard', info.board);
    })
    .catch(() => {});

  function currentType() {
    return typeSelect && typeSelect.value === 'filesystem' ? 'filesystem' : 'firmware';
  }

  function updateUploadLabel() {
    uploadBtn.textContent = currentType() === 'filesystem' ? 'Upload Filesystem' : 'Upload Firmware';
  }

  if (typeSelect) {
    typeSelect.addEventListener('change', updateUploadLabel);
    updateUploadLabel();
  }

  chooseBtn.addEventListener('click', (e) => {
    e.stopPropagation();
    fileInput.click();
  });

  dropZone.addEventListener('dragover', (e) => {
    e.preventDefault();
    dropZone.classList.add('drag-over');
  });
  dropZone.addEventListener('dragleave', () => dropZone.classList.remove('drag-over'));
  dropZone.addEventListener('drop', (e) => {
    e.preventDefault();
    dropZone.classList.remove('drag-over');
    const file = e.dataTransfer.files[0];
    if (file) selectFile(file);
  });

  fileInput.addEventListener('change', () => {
    if (fileInput.files[0]) selectFile(fileInput.files[0]);
  });

  function selectFile(file) {
    if (!file.name.endsWith('.bin')) {
      setOtaStatus('Please select a .bin file.', 'error');
      return;
    }
    fileInput._selectedFile = file;
    fileNameEl.textContent = file.name + ' (' + (file.size / 1024).toFixed(1) + ' KB)';
    dropZone.classList.add('file-selected');
    uploadBtn.disabled = false;
    setOtaStatus('');
  }

  uploadBtn.addEventListener('click', () => {
    const file = fileInput._selectedFile;
    if (!file) return;

    const uploadType = currentType();
    // Single endpoint for both; ?mode= selects the app or LittleFS partition.
    const formData = new FormData();
    formData.append('firmware', file, file.name);

    const xhr = new XMLHttpRequest();

    xhr.upload.addEventListener('progress', (e) => {
      if (e.lengthComputable) {
        const pct = Math.round((e.loaded / e.total) * 100);
        progressBar.style.width = pct + '%';
        progressLbl.textContent = pct + '%';
      }
    });

    xhr.addEventListener('load', () => {
      try {
        const resp = JSON.parse(xhr.responseText);
        if (resp.status === 'ok') {
          setOtaStatus(resp.message || 'Update complete. Device is rebooting...', 'success');
          uploadBtn.disabled = true;
        } else {
          setOtaStatus('Update failed: ' + (resp.message || 'Unknown error'), 'error');
          resetProgress();
        }
      } catch (_) {
        setOtaStatus('Unexpected response from device.', 'error');
        resetProgress();
      }
    });

    xhr.addEventListener('error', () => {
      // A network error here is expected if the device reboots before replying.
      setOtaStatus('Update sent. Device may be rebooting — please wait and reconnect.', 'success');
    });

    progressWrap.style.display = 'block';
    progressBar.style.width = '0%';
    progressLbl.textContent = '0%';
    uploadBtn.disabled = true;
    setOtaStatus('Uploading...');

    xhr.open('POST', '/api/ota-update?mode=' + uploadType);
    xhr.send(formData);
  });

  function setOtaStatus(msg, type) {
    statusEl.textContent = msg;
    statusEl.className = 'ota-status' + (type ? ' ' + type : '');
  }

  function resetProgress() {
    progressBar.style.width = '0%';
    progressLbl.textContent = '0%';
    progressWrap.style.display = 'none';
    uploadBtn.disabled = false;
  }
}

function applySpeedUnitLabels(useMPH) {
  const label = useMPH ? 'MPH' : 'KMH';
  ['speedUnit', 'speedOffsetUnit', 'measuredSpeedUnit'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.textContent = label;
  });
}

// Single source of truth for the cluster unit: keeps the Configuration toggle,
// the Calibration Builder toggle, all unit labels and the cal-builder unit text
// in lock-step so the two "Cluster in MPH" switches can never disagree.
function applyMphState(useMPH) {
  useMPH = !!useMPH;
  applySpeedUnitLabels(useMPH);
  const cfg = document.getElementById('convertToMPH');
  if (cfg) cfg.checked = useMPH;
  const calMph = document.getElementById('calConvertToMPH');
  if (calMph) calMph.checked = useMPH;
  const calUnitEl = document.getElementById('calUnitLabel');
  if (calUnitEl) calUnitEl.textContent = useMPH ? 'mph' : 'km/h';
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
    document.getElementById('sweepSpeed').value = data.sweepSpeed ?? 18;
    document.getElementById('sweepSpeed-display').textContent = data.sweepSpeed ?? 18;
    document.getElementById('stepSpeed').value = data.stepSpeed ?? 17;
    document.getElementById('stepSpeed-display').textContent = data.stepSpeed ?? 17;
    document.getElementById('stepRPM').value = data.stepRPM ?? 14;
    document.getElementById('stepRPM-display').textContent = data.stepRPM ?? 14;
    document.getElementById('coilType').checked = data.coilType || false;
    document.getElementById('convertToMPH').checked = data.convertToMPH || false;
    applyMphState(!!data.convertToMPH);
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
    applyOffsetCurveVisibility(!!data.useSpeedOffsetCurve);

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
    const gpsRateSelect = document.getElementById('gpsRateSelect');
    if (gpsRateSelect) {
      const savedGpsRate = String(data.gpsUpdateRateHz ?? 1);
      gpsRateSelect.value = Array.from(gpsRateSelect.options).some(option => option.value === savedGpsRate)
        ? savedGpsRate
        : '1';
    }
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
    const tempDutyDisp = document.getElementById('tempDutyCycle-display');
    if (tempDutyDisp) tempDutyDisp.textContent = data.tempDutyCycle || 0;
    document.getElementById('maxRPM').value = data.maxRPM || 230;
    document.getElementById('maxRPM-display').textContent = data.maxRPM || 230;
    document.getElementById('clusterRPMLimit').value = data.clusterRPMLimit || 7000;
    document.getElementById('clusterRPMLimit-display').textContent = data.clusterRPMLimit || 7000;
    const analyzerModeEl = document.getElementById('analyzerMode');
    if (analyzerModeEl) analyzerModeEl.checked = data.analyzerMode || false;
    const analyzerSerialEl = document.getElementById('analyzerSerial');
    if (analyzerSerialEl) analyzerSerialEl.checked = data.analyzerSerial || false;

    // Closed-loop feedback (PID) settings
    const feedbackEnableEl = document.getElementById('feedbackEnable');
    if (feedbackEnableEl) feedbackEnableEl.checked = data.feedbackEnable || false;
    const reverseDirectionEl = document.getElementById('reverseDirection');
    if (reverseDirectionEl) reverseDirectionEl.checked = data.reverseDirection || false;
    const feedbackMinSpeedEl = document.getElementById('feedbackMinSpeed');
    if (feedbackMinSpeedEl) {
      feedbackMinSpeedEl.value = data.feedbackMinSpeed ?? 40;
      const disp = document.getElementById('feedbackMinSpeed-display');
      if (disp) disp.textContent = data.feedbackMinSpeed ?? 40;
    }
    const pidKpEl = document.getElementById('pidKp');
    if (pidKpEl) pidKpEl.value = data.pidKp ?? 0.15;
    const pidKiEl = document.getElementById('pidKi');
    if (pidKiEl) pidKiEl.value = data.pidKi ?? 1.3;
    const pidKdEl = document.getElementById('pidKd');
    if (pidKdEl) pidKdEl.value = data.pidKd ?? 0;
    const feedbackDeadbandEl = document.getElementById('feedbackDeadband');
    if (feedbackDeadbandEl) {
      feedbackDeadbandEl.value = data.feedbackDeadband ?? 1.5;
      const disp = document.getElementById('feedbackDeadband-display');
      if (disp) disp.textContent = data.feedbackDeadband ?? 1.5;
    }

    // Speed type dropdown - map speedType to dropdown options
    let speedTypeValue = 'Hall';  // default
    if (data.speedType === 'ECU') speedTypeValue = 'ECU';
    else if (data.speedType === 'ABS') speedTypeValue = 'ABS';
    else if (data.speedType === 'DSG') speedTypeValue = 'DSG';
    else if (data.speedType === 'TP2.0') speedTypeValue = 'TP2.0';
    else if (data.speedType === 'UDS') speedTypeValue = 'UDS';
    else if (data.speedType === 'GPS') speedTypeValue = 'GPS';
    else if (data.speedType === 'Custom CAN') speedTypeValue = 'Custom CAN';
    document.getElementById('speedSource').value = speedTypeValue;
    const customCANCard = document.getElementById('customCANInputCard');
    if (customCANCard) customCANCard.style.display = speedTypeValue === 'Custom CAN' ? '' : 'none';

    // Aftermarket / Custom CAN input settings
    document.getElementById('aftermarketSpeedID').value = (data.aftermarketSpeedID || 0).toString(16).toUpperCase();
    document.getElementById('aftermarketSpeedLowByte').value = data.aftermarketSpeedLowByte ?? 0;
    document.getElementById('aftermarketSpeedHighByte').value = data.aftermarketSpeedHighByte ?? 1;
    document.getElementById('aftermarketSpeedLittleEndian').value = (data.aftermarketSpeedLittleEndian !== false) ? 'true' : 'false';
    document.getElementById('aftermarketSpeedScale').value = (data.aftermarketSpeedScale ?? 1.0).toFixed(3);
    document.getElementById('aftermarketSpeedOffset').value = data.aftermarketSpeedOffset ?? 0;

    // RPM source dropdown
    document.getElementById('rpmSource').value = (data.rpmType === 'CAN') ? 'CAN' : 'Hall';

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
    const motorDutyEl = document.getElementById('motorDuty');
    if (motorDutyEl) motorDutyEl.textContent = data.appliedDutyCycle ?? '--';
    if (data.convertToMPH !== undefined) {
      applyMphState(!!data.convertToMPH);
    }
    
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
    if (document.getElementById('liveSpeed')) {
      document.getElementById('liveSpeed').textContent = data.vehicleSpeed || '--';
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
    if (document.getElementById('liveTP20Speed')) {
      document.getElementById('liveTP20Speed').textContent = data.tp20Speed !== undefined ? data.tp20Speed : '--';
    }
    if (document.getElementById('liveUDSSpeed')) {
      document.getElementById('liveUDSSpeed').textContent = data.udsSpeed !== undefined ? data.udsSpeed : '--';
    }
    if (document.getElementById('liveGPSSpeed')) {
      document.getElementById('liveGPSSpeed').textContent = data.gpsSpeed || '--';
    }
    if (document.getElementById('liveAftermarketSpeed')) {
      document.getElementById('liveAftermarketSpeed').textContent = data.aftermarketSpeed !== undefined ? data.aftermarketSpeed : '--';
    }
    if (document.getElementById('liveAftermarketSpeedCard')) {
      document.getElementById('liveAftermarketSpeedCard').textContent = data.aftermarketSpeed !== undefined ? data.aftermarketSpeed : '--';
    }
    if (document.getElementById('tempDutyCycle-display')) {
      document.getElementById('tempDutyCycle-display').textContent = data.tempDutyCycle || 0;
    }
    updateCalDutyReadout(data.tempDutyCycle);

    if (document.getElementById('liveGPSStatus')) {
      if (data.hasGPS) {
        document.getElementById('liveGPSStatus').textContent = `Connected (${data.gpsSatellites} sats)`;
      } else if (data.gpsUnavailable) {
        document.getElementById('liveGPSStatus').textContent = 'Not Available';
      } else {
        document.getElementById('liveGPSStatus').textContent = 'Not Connected';
      }
    }

    // GPS Frequency card
    if (document.getElementById('liveGPSFrequency')) {
      const rawFreq = data.gpsFrequency;
      const freq = (typeof rawFreq === 'number') ? rawFreq : Number(rawFreq);
      document.getElementById('liveGPSFrequency').textContent = Number.isFinite(freq) ? freq.toFixed(2) : '--';
    }

    // GPS auto rate countdown - show in the same green response area used by
    // the Set button, but only when the user isn't actively reading their
    // own click feedback.
    const gpsRateResponseEl = document.getElementById('gpsRateResponse');
    if (gpsRateResponseEl && (!window.gpsRateUserMessageUntil || Date.now() > window.gpsRateUserMessageUntil)) {
      const secs = data.gpsAutoApplySecs;
      if (typeof secs === 'number' && secs >= 0) {
        gpsRateResponseEl.style.color = '#007a3d';
        if (secs === 0) {
          gpsRateResponseEl.textContent = 'Auto-applying saved rate...';
        } else {
          gpsRateResponseEl.textContent = `Auto-apply in ${secs}s (waiting for satellite lock + 20s).`;
        }
      } else if (gpsRateResponseEl.textContent.startsWith('Auto-apply') || gpsRateResponseEl.textContent.startsWith('Auto-applying')) {
        gpsRateResponseEl.textContent = '';
      }
    }

    updateSpeedOffsetStatus(data.speedOffsetType, data.currentSpeedOffset);

    // System status (read-only, not settings) — tick/cross text only, neutral
    // badge (OpenHaldex style: never colour the whole badge red/green).
    const canEl = document.getElementById('canStatus');
    canEl.textContent = data.hasCAN ? 'CAN: ✓' : 'CAN: ✗';
    canEl.classList.remove('connected', 'error');
    const broadcastEl = document.getElementById('broadcastStatus');
    broadcastEl.textContent = data.broadcastSpeedEnabled ? 'Broadcast: ✓' : 'Broadcast: ✗';
    broadcastEl.classList.remove('connected', 'error');
    document.getElementById('canPresent').textContent = data.hasCAN ? 'Healthy' : 'Not Healthy';

    if (document.getElementById('liveBroadcastSpeedValue')) {
      const suffix = data.broadcastSpeedEnabled ? '' : ' (disabled)';
      document.getElementById('liveBroadcastSpeedValue').textContent = `${data.broadcastSpeedValue || 0}${suffix}`;
    }
    
    // GPS status in dashboard
    if (document.getElementById('gpsPresent')) {
      if (data.hasGPS) {
        document.getElementById('gpsPresent').textContent = `Connected (${data.gpsSatellites} sats)`;
      } else if (data.gpsUnavailable) {
        document.getElementById('gpsPresent').textContent = 'Not Available';
      } else {
        document.getElementById('gpsPresent').textContent = 'Not Connected';
      }
    }

    // Closed-loop feedback (PID) live status
    updateFeedbackStatus(data);

    // Diagnostics: colour the active outputs and the input driving them.
    updateDiagHighlights(data);

    // Live calibration-curve marker. In cal mode the achieved speed is on the
    // needle (unknown here), so ride the curve at the jogged duty; otherwise use
    // the incoming vehicle speed against the applied duty.
    if (data.testCal) {
      setCalCurrentPoint(data.tempDutyCycle, curveSpeedAt(data.tempDutyCycle));
    } else {
      setCalCurrentPoint(data.appliedDutyCycle, data.vehicleSpeed);
    }

  } catch (error) {
    console.log('Error fetching status:', error);
  }
}

function updateFeedbackStatus(data) {
  const fbOn = !!data.feedbackEnable;

  // Feedback readouts have three states:
  //   available -> show the measured value / PID trim
  //   missing   -> motor running but no feedback signal (legacy PCB): show "N/A"
  //   unknown   -> not seen yet and motor idle: leave "--"
  const fbValue = (numeric) => {
    if (data.feedbackAvailable) return numeric;
    if (data.feedbackMissing) return 'N/A';
    return '--';
  };

  const speedNumeric = data.measuredSpeed !== undefined ? String(data.measuredSpeed) : '--';
  const trimNumeric = (data.pidCorrection !== undefined && data.pidCorrection !== null)
    ? ((data.pidCorrection > 0 ? '+' : '') + data.pidCorrection)
    : '--';
  const speedTxt = fbValue(speedNumeric);
  const trimTxt = fbValue(trimNumeric);
  const freqTxt = (typeof data.measuredFreqHz === 'number')
    ? data.measuredFreqHz.toFixed(1)
    : '--';

  const setTxt = (id, txt) => {
    const el = document.getElementById(id);
    if (el) el.textContent = txt;
  };

  // Dashboard gauges (mirror the base: always show the measured values)
  setTxt('feedbackState', fbOn ? 'Enabled' : 'Off');
  setTxt('measuredSpeed', speedTxt);
  setTxt('pidCorrection', trimTxt);
  setTxt('measuredFreqHz', freqTxt);

  // Advanced feedback card live readout
  setTxt('liveMeasuredSpeed', fbOn ? speedTxt : '--');
  setTxt('livePidCorrection', fbOn ? trimTxt : '--');
  setTxt('liveMeasuredFreqHz', freqTxt);

  // Live Data - All Inputs (motor feedback details)
  setTxt('liveAllMotorSpeed', speedTxt);
  setTxt('liveAllPidTrim', fbOn ? trimTxt : '--');
  setTxt('liveAllMotorFreq', freqTxt);
}

// Colour the Diagnostics live-data so it's obvious which outputs are active and
// what is driving them: the Final RPM / Final Speed light up orange when driven,
// together with the input actually feeding them (or nothing extra in Test Mode).
function updateDiagHighlights(data) {
  const speedSourceMap = {
    'Hall': 'liveHallSpeed', 'ECU': 'liveECUSpeed', 'ABS': 'liveABSSpeed',
    'DSG': 'liveDSGSpeed', 'TP2.0': 'liveTP20Speed', 'UDS': 'liveUDSSpeed',
    'GPS': 'liveGPSSpeed', 'Custom CAN': 'liveAftermarketSpeed'
  };
  const rpmSourceMap = { 'CAN': 'liveCANRPM', 'Hall': 'liveHallRPM' };

  const setActive = (id, on) => {
    const el = document.getElementById(id);
    if (el) el.classList.toggle('live-active', !!on);
  };

  // Clear every candidate first so stale highlights don't linger.
  ['liveSpeed', 'liveRPM', 'liveHallRPM', 'liveCANRPM',
    'liveHallSpeed', 'liveECUSpeed', 'liveABSSpeed', 'liveDSGSpeed',
    'liveTP20Speed', 'liveUDSSpeed', 'liveGPSSpeed', 'liveAftermarketSpeed']
    .forEach(id => setActive(id, false));

  // Speed channel
  const speedDriven = !!data.testSpeedo || Number(data.vehicleSpeed) > 0;
  if (speedDriven) {
    setActive('liveSpeed', true);
    if (!data.testSpeedo) {
      const src = speedSourceMap[data.activeSpeedSource];
      if (src) setActive(src, true);
    }
  }

  // RPM channel
  const rpmDriven = !!data.testRPM || Number(data.vehicleRPM) > 0;
  if (rpmDriven) {
    setActive('liveRPM', true);
    if (!data.testRPM) {
      const src = rpmSourceMap[data.activeRpmSource];
      if (src) setActive(src, true);
    }
  }
}

function pushControl(key, value) {
  return fetch('/api/control', {
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

// ===== CALIBRATION BUILDER =====
const CAL_PWM_MAX = 4095;
const CAL_CHIP_MAX = 300;   // capture targets available up to 300 km/h
let calSelectedSpeed = 0;
let calChipsMaxSpeed = -1;
let lastCalPointCount = 0;  // points in the current builder state (save guard)

function initCalBuilder() {
  // Jog steppers with press-and-hold repeat + acceleration
  document.querySelectorAll('.stepper-btn[data-jog]').forEach((btn) => {
    const delta = parseInt(btn.dataset.jog, 10);
    attachHold(btn, () => calJog(delta));
  });

  const targetInput = document.getElementById('calTargetSpeed');
  if (targetInput) {
    targetInput.addEventListener('input', () => setCalTarget(parseInt(targetInput.value, 10) || 0, false));
  }

  const captureBtn = document.getElementById('calCaptureBtn');
  if (captureBtn) captureBtn.addEventListener('click', calCapture);

  const nameEl = document.getElementById('calName');
  if (nameEl) {
    nameEl.addEventListener('change', () => calPost({ op: 'setName', name: nameEl.value }).catch(() => {}));
  }

  // Cal-page "Cluster in MPH" toggle — mirrors and drives the Configuration one.
  const calMphEl = document.getElementById('calConvertToMPH');
  if (calMphEl) {
    calMphEl.addEventListener('change', () => {
      applyMphState(calMphEl.checked);
      pushControl('convertToMPH', calMphEl.checked);
    });
  }

  bindCal('calApplyBtn', () => calPost({ op: 'apply' }).then((s) => {
    applyCalState(s);
    refreshCalibrationsSelect();
    fetchCalCurve();
    showNotification('Calibration generated and applied');
  }));

  bindCal('calSaveBtn', () => {
    if (lastCalPointCount < 2) {
      showNotification('Capture at least 2 points before saving', 'error');
      return;
    }
    // A saved custom cal shows up as the value=200 option; warn before clobbering it.
    const sel = document.getElementById('motorCalibration');
    const exists = sel && [...sel.options].some((o) => o.value === '200');
    if (exists && !confirm('A custom calibration is already saved on the device. Overwrite it?')) {
      return;
    }
    calPost({ op: 'save' }).then((s) => {
      applyCalState(s);
      refreshCalibrationsSelect();
      fetchCalCurve();
      showNotification('Calibration saved to device');
    });
  });

  bindCal('calClearBtn', () => {
    if (!confirm('Clear all captured calibration points?')) return;
    calPost({ op: 'clearPoints' }).then(applyCalState).then(() => showNotification('Points cleared'));
  });

  bindCal('calExportTextBtn', () => calPost({ op: 'export' }).then((r) => {
    const text = r.json || '';
    document.getElementById('calText').value = text;
    const nm = document.getElementById('calName');
    const base = (nm && nm.value.trim()) || 'calibration';
    const fname = base.replace(/[^a-z0-9._-]+/gi, '_') + '.txt';
    downloadTextFile(fname, text);
    showNotification('Exported to text file');
  }));

  bindCal('calExportCArrayBtn', () => calPost({ op: 'export' }).then((r) => {
    const text = r.carray || '';
    document.getElementById('calText').value = text;
    const nm = document.getElementById('calName');
    const base = (nm && nm.value.trim()) || 'calibration';
    const fname = base.replace(/[^a-z0-9._-]+/gi, '_') + '.h';
    downloadTextFile(fname, text);
    showNotification('Exported to C array file');
  }));

  bindCal('calImportBtn', () => {
    const fileEl = document.getElementById('calImportFile');
    if (fileEl) fileEl.click();
  });

  const importFileEl = document.getElementById('calImportFile');
  if (importFileEl) {
    importFileEl.addEventListener('change', () => {
      const file = importFileEl.files[0];
      if (!file) return;
      const reader = new FileReader();
      reader.onload = () => {
        const text = String(reader.result || '').trim();
        document.getElementById('calText').value = text;
        if (text) calImportText(text);
      };
      reader.readAsText(file);
      importFileEl.value = '';   // allow re-picking the same file
    });
  }
}

function calImportText(text) {
  calPost({ op: 'import', json: text }).then((s) => {
    applyCalState(s);
    refreshCalibrationsSelect();
    fetchCalCurve();
    showNotification('Calibration imported');
  });
}

// Trigger a browser download of the given text as a file.
function downloadTextFile(filename, text) {
  const blob = new Blob([text], { type: 'text/plain' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function bindCal(id, fn) {
  const el = document.getElementById(id);
  if (el) el.addEventListener('click', () => { try { fn(); } catch (e) { console.log(e); } });
}

// Press-and-hold helper: fires immediately, then repeats faster while held.
function attachHold(el, fn) {
  let timer = null;
  let delay = 320;
  const stop = () => { if (timer) { clearTimeout(timer); timer = null; } };
  const start = (e) => {
    e.preventDefault();
    fn();
    delay = 320;
    const tick = () => { fn(); delay = Math.max(60, delay * 0.8); timer = setTimeout(tick, delay); };
    timer = setTimeout(tick, delay);
  };
  el.addEventListener('pointerdown', start);
  el.addEventListener('pointerup', stop);
  el.addEventListener('pointerleave', stop);
  el.addEventListener('pointercancel', stop);
}

async function calPost(body) {
  const r = await fetch('/api/cal', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  });
  if (!r.ok) {
    const err = await r.json().catch(() => ({}));
    showNotification('Cal error: ' + (err.error || ('HTTP ' + r.status)), 'error');
    throw new Error(err.error || r.status);
  }
  return r.json();
}

async function refreshCalState() {
  try {
    const r = await fetch('/api/cal');
    applyCalState(await r.json());
  } catch (e) {
    console.log('Error fetching cal state:', e);
  }
}

async function calJog(delta) {
  try {
    const s = await calPost({ op: 'jog', delta });
    updateCalDutyReadout(s.duty);
  } catch (e) { /* already notified */ }
}

async function calCapture() {
  try {
    const s = await calPost({ op: 'addPoint', speed: calSelectedSpeed });
    applyCalState(s);
    showNotification('Captured ' + calSelectedSpeed + ' @ duty ' + s.duty);
  } catch (e) { /* already notified */ }
}

function updateCalDutyReadout(duty) {
  duty = duty || 0;
  const nowEl = document.getElementById('calDutyNow');
  if (!nowEl) return;
  nowEl.textContent = duty;
  const pctEl = document.getElementById('calDutyPct');
  const capDutyEl = document.getElementById('calCaptureDuty');
  if (pctEl) pctEl.textContent = (duty / CAL_PWM_MAX * 100).toFixed(1);
  if (capDutyEl) capDutyEl.textContent = duty;
}

function applyCalState(state) {
  if (!state) return;
  updateCalDutyReadout(state.duty);

  const unit = state.unit === 'mph' ? 'mph' : 'km/h';
  const unitEl = document.getElementById('calUnitLabel');
  if (unitEl) unitEl.textContent = unit;

  // Mirror the firmware's runtime cluster unit onto both MPH toggles.
  if (typeof state.convertToMPH === 'boolean') applyMphState(state.convertToMPH);

  // Name (don't clobber while the user is editing it)
  const nameEl = document.getElementById('calName');
  if (nameEl && document.activeElement !== nameEl) nameEl.value = state.name || '';

  buildCalChips(state.maxSpeed || 200);
  renderCalPoints(state.points || []);
  lastCalPointCount = state.count ?? (state.points || []).length;
  scheduleCalDraw();
}

function buildCalChips(maxSpeed) {
  const chipMax = Math.max(maxSpeed || 0, CAL_CHIP_MAX);
  if (chipMax === calChipsMaxSpeed) return;
  calChipsMaxSpeed = chipMax;
  const grid = document.getElementById('calTargetChips');
  if (!grid) return;
  grid.innerHTML = '';
  for (let s = 0; s <= chipMax; s += 10) {
    const chip = document.createElement('button');
    chip.className = 'chip';
    chip.textContent = s;
    chip.dataset.speed = s;
    chip.addEventListener('click', () => setCalTarget(s, true));
    grid.appendChild(chip);
  }
  highlightCalChip();
}

function setCalTarget(speed, fromChip) {
  calSelectedSpeed = speed;
  const capEl = document.getElementById('calCaptureSpeed');
  if (capEl) capEl.textContent = speed;
  const input = document.getElementById('calTargetSpeed');
  if (input && fromChip) input.value = speed;
  highlightCalChip();
}

function highlightCalChip() {
  document.querySelectorAll('#calTargetChips .chip').forEach((c) => {
    c.classList.toggle('active', parseInt(c.dataset.speed, 10) === calSelectedSpeed);
  });
}

function renderCalPoints(points) {
  const list = document.getElementById('calPointsList');
  if (!list) return;
  if (!points.length) {
    list.innerHTML = '<p class="hint">No points captured yet.</p>';
    return;
  }
  let html = '<div class="cal-point-head"><span>Speed</span><span>Duty</span><span></span></div>';
  points.forEach((p, i) => {
    html += '<div class="cal-point-row">' +
              '<span>' + p.speed + '</span>' +
              '<span>' + p.duty + '</span>' +
              '<button class="cal-del" data-index="' + i + '">✕</button>' +
            '</div>';
  });
  list.innerHTML = html;
  list.querySelectorAll('.cal-del').forEach((btn) => {
    btn.addEventListener('click', () => {
      const index = parseInt(btn.dataset.index, 10);
      calPost({ op: 'deletePoint', index }).then(applyCalState).catch(() => {});
    });
  });
}

// Refresh the Configuration dropdown so the custom slot appears/updates,
// then select it so the user can see it is active.
function refreshCalibrationsSelect() {
  fetchCalibrations().then(() => {
    const sel = document.getElementById('motorCalibration');
    if (sel && [...sel.options].some((o) => o.value === '200')) {
      sel.value = '200';
    }
  });
}

// ===== CALIBRATION CURVE GRAPH =====
let calCurveData = null;          // { speed:[], duty:[], pwmMax, maxSpeed } from /api/calcurve
let calCurrentPoint = null;       // { duty, speed } live operating point
let calDrawPending = false;

async function fetchCalCurve() {
  try {
    const r = await fetch('/api/calcurve');
    calCurveData = await r.json();
  } catch (e) {
    console.log('Error fetching cal curve:', e);
  }
  scheduleCalDraw();
}

function setCalCurrentPoint(duty, speed) {
  calCurrentPoint = { duty: Number(duty) || 0, speed: Number(speed) || 0 };
  scheduleCalDraw();
}

// Interpolate the curve's speed at a given hardware duty (used in cal mode,
// where the true achieved speed isn't reported by the firmware).
function curveSpeedAt(duty) {
  if (!calCurveData || !Array.isArray(calCurveData.duty) || calCurveData.duty.length < 2) return 0;
  const d = calCurveData.duty, s = calCurveData.speed;
  if (duty <= d[0]) return s[0];
  for (let i = 1; i < d.length; i++) {
    if (duty <= d[i]) {
      const span = d[i] - d[i - 1];
      const frac = span > 0 ? (duty - d[i - 1]) / span : 0;
      return Math.round(s[i - 1] + frac * (s[i] - s[i - 1]));
    }
  }
  return s[s.length - 1];
}

// Coalesce redraws to one per animation frame (the status poll fires often).
function scheduleCalDraw() {
  if (calDrawPending) return;
  calDrawPending = true;
  requestAnimationFrame(() => { calDrawPending = false; drawCalCurve(); });
}

function drawCalCurve() {
  const canvas = document.getElementById('calCurveCanvas');
  if (!canvas || !canvas.clientWidth) return;   // not laid out yet (hidden tab)

  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth;
  const cssH = Math.round(cssW * 0.6);
  canvas.style.height = cssH + 'px';
  if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
    canvas.width = Math.round(cssW * dpr);
    canvas.height = Math.round(cssH * dpr);
  }

  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);

  const style = getComputedStyle(document.documentElement);
  const col = (name, fallback) => (style.getPropertyValue(name).trim() || fallback);
  const primary = col('--primary', '#00D9FF');
  const secondary = col('--secondary', '#FF6B35');
  const border = col('--border', '#30363D');
  const textDim = col('--text-dim', '#8B949E');

  const padL = 44, padR = 12, padT = 12, padB = 30;
  const plotW = cssW - padL - padR;
  const plotH = cssH - padT - padB;

  const pwmMax = (calCurveData && calCurveData.pwmMax) || 384;
  // Stretch the x-axis across only the duty range the calibration actually uses
  // (the curve tops out well below full-scale), so the trace fills the width.
  let dutyMax = 1;
  if (calCurveData && Array.isArray(calCurveData.duty) && calCurveData.duty.length) {
    dutyMax = calCurveData.duty[calCurveData.duty.length - 1] || 1;
  }
  if (calCurrentPoint && calCurrentPoint.duty > dutyMax) dutyMax = calCurrentPoint.duty;
  dutyMax = Math.max(1, Math.ceil(dutyMax / 100) * 100);

  let speedMax = (calCurveData && calCurveData.maxSpeed) || 200;
  if (calCurrentPoint && calCurrentPoint.speed > speedMax) {
    speedMax = Math.ceil(calCurrentPoint.speed / 20) * 20;   // grow if the live point overshoots
  }
  if (speedMax < 1) speedMax = 1;

  const xOf = (duty) => padL + (Math.max(0, Math.min(duty, dutyMax)) / dutyMax) * plotW;
  const yOf = (speed) => padT + plotH - (Math.max(0, Math.min(speed, speedMax)) / speedMax) * plotH;

  ctx.font = '10px -apple-system, "Segoe UI", Arial, sans-serif';
  ctx.strokeStyle = border;
  ctx.lineWidth = 1;

  // Horizontal grid (speed) + Y labels
  ctx.fillStyle = textDim;
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  const ySteps = 4;
  for (let i = 0; i <= ySteps; i++) {
    const sp = (speedMax / ySteps) * i;
    const y = yOf(sp);
    ctx.globalAlpha = 0.35;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(cssW - padR, y); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillText(String(Math.round(sp)), padL - 6, y);
  }

  // Vertical grid (duty %) + X labels
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  const xSteps = 5;
  for (let i = 0; i <= xSteps; i++) {
    const x = padL + (plotW / xSteps) * i;
    ctx.globalAlpha = 0.35;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.fillText(String(Math.round((i / xSteps) * dutyMax)), x, padT + plotH + 6);
  }

  // Axis title
  ctx.fillStyle = textDim;
  ctx.textAlign = 'center';
  ctx.textBaseline = 'bottom';
  ctx.fillText('Motor duty', padL + plotW / 2, cssH);

  // Faint calibration curve
  if (calCurveData && Array.isArray(calCurveData.duty) && calCurveData.duty.length > 1) {
    ctx.strokeStyle = primary;
    ctx.globalAlpha = 0.5;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < calCurveData.duty.length; i++) {
      const x = xOf(calCurveData.duty[i]);
      const y = yOf(calCurveData.speed[i]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // Anchor marks of the ACTIVE calibration — always sit on the blue curve.
  if (calCurveData && Array.isArray(calCurveData.anchorDuty) && calCurveData.anchorDuty.length) {
    ctx.fillStyle = col('--success', '#2EA043');
    for (let i = 0; i < calCurveData.anchorDuty.length; i++) {
      ctx.beginPath();
      ctx.arc(xOf(calCurveData.anchorDuty[i]), yOf(calCurveData.anchorSpeed[i]), 3.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Current operating point + crosshair
  if (calCurrentPoint) {
    const x = xOf(calCurrentPoint.duty);
    const y = yOf(calCurrentPoint.speed);

    ctx.strokeStyle = secondary;
    ctx.globalAlpha = 0.5;
    ctx.setLineDash([4, 4]);
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(x, y); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x, padT + plotH); ctx.lineTo(x, y); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;

    ctx.fillStyle = secondary;
    ctx.beginPath();
    ctx.arc(x, y, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();

    const label = calCurrentPoint.speed + ' @ ' + Math.round(calCurrentPoint.duty) + ' duty';
    ctx.font = '11px -apple-system, "Segoe UI", Arial, sans-serif';
    ctx.fillStyle = secondary;
    ctx.textBaseline = 'bottom';
    const rightSide = x > padL + plotW * 0.7;
    ctx.textAlign = rightSide ? 'right' : 'left';
    ctx.fillText(label, rightSide ? x - 8 : x + 8, y - 6);
  }
}
