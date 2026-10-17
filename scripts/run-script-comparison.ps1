# Offline only. Rebuild the actual firmware smoother before replaying scripts.
param([string]$FirmwareRoot='C:/Users/wangyu/Desktop/ctw_esp32s3_0909')
$ErrorActionPreference='Stop'
$comparisonRoot=Split-Path $PSScriptRoot -Parent
Push-Location $FirmwareRoot
try {
  conda run -n base python scripts/build_ruckig_script_reference.py
  if($LASTEXITCODE -ne 0){throw 'Firmware host build failed'}
} finally {Pop-Location}
Push-Location $comparisonRoot
try {
  New-Item -ItemType Directory -Force work/script-comparison | Out-Null
  $comparisonObjects=0..13 | ForEach-Object {Join-Path $FirmwareRoot "tmp/ruckig-script-reference/$_.o"}
  & g++ -std=gnu++17 -O3 -fno-fast-math -ffp-contract=off -DCTW_ENABLE_RUCKIG=1 -I "$FirmwareRoot/include" scripts/compare-script-ruckig.cpp @comparisonObjects -o work/script-comparison/replay.exe
  if($LASTEXITCODE -ne 0){throw 'Replay adapter build failed'}
  $env:DYNAMICS_STEP_MS='1';$env:DERIVATIVE_H='0.00005'
  foreach($interpolation in @('linear','pchip')) {
    $env:SCRIPT_INTERPOLATION=$interpolation
    $env:COMPARISON_OUT=if($interpolation -eq 'linear'){'work/script-comparison'}else{'work/script-comparison-pchip'}
    foreach($stage in @('prepare','replay','analyze')) {
      node scripts/compare-script-paths.mjs $stage
      if($LASTEXITCODE -ne 0){throw "Comparison $interpolation $stage failed"}
    }
  }
  conda run -n base python scripts/plot-script-comparison.py
  if($LASTEXITCODE -ne 0){throw 'Plot failed'}
} finally {Pop-Location}
