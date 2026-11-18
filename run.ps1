<#
.SYNOPSIS
    Build the guidance core if needed, fly a landing, and replay it in real time.

.EXAMPLE
    .\run.ps1
    .\run.ps1 -Scenario entry -Speed 4    # the whole return from separation, 62 km
    .\run.ps1 -ZoomTo 100                 # adaptive camera, tighter close-up
    .\run.ps1 -Follow 200                 # chase camera all the way instead
    .\run.ps1 -R0 "3000,0,10000" -V0 "-60,0,-300" -Nodes 41 -TfGuess 70
    .\run.ps1 -Renderer rerun             # GPU viewer instead of matplotlib
    .\run.ps1 -Speed 0.5 -MisalignDeg 0.2 -ActuatorTau 0.05
    .\run.ps1 -Build                      # only (re)build and run the unit tests
#>
[CmdletBinding()]
param(
    [string]$Out = "out",
    [string]$R0,
    [string]$V0,
    [double]$Speed = 1.0,
    # vehicle preset: pdg's own Mars-case stage, or a Falcon-9-class one
    [ValidateSet("default", "f9")]
    [string]$Preset = "default",
    [ValidateSet("mpl", "rerun")]
    [string]$Renderer = "mpl",
    [double]$Follow = 0.0,
    [double]$ZoomAt = 2000.0,
    [double]$ZoomTo = 200.0,
    [double]$ZoomPause = 1.5,
    # donor scenarios: the full return from separation, with an atmosphere
    [ValidateSet("entry", "terminal", "aero_offset", "asds_night", "chaos")]
    [string]$Scenario,
    # the solo first-stage RTLS test flight: liftoff, boostback, return, landing
    [switch]$Rtls,
    [double]$Gamma = 0.0,
    [int]$AimPasses = 4,
    [int]$Seed = 1,
    [int]$Nodes = 0,
    [double]$TfGuess = 0.0,
    [double]$VehicleScale = 1.0,
    [double]$ThrustScale = 1.0,
    [double]$MisalignDeg = 0.0,
    [double]$ActuatorTau = 0.0,
    [switch]$Build,
    [switch]$NoViewer,
    [string]$Save
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$venv = Join-Path $root ".venv\Scripts"
$python = Join-Path $venv "python.exe"
$cmake = Join-Path $venv "cmake.exe"
$ninja = Join-Path $venv "ninja.exe"
$ctest = Join-Path $venv "ctest.exe"

if (-not (Test-Path $python)) {
    throw "No virtualenv at $venv. Run setup.ps1 first."
}

# ---- locate the MSVC environment ------------------------------------------
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found -- is Visual Studio Build Tools installed?" }
$vsPath = & $vswhere -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath -latest
if (-not $vsPath) { throw "No MSVC C++ toolchain found. Install the 'Desktop development with C++' workload." }
$vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"

function Invoke-InMsvcEnv([string]$Command) {
    # vcvars64.bat has to run in the same cmd process as the tool it sets up.
    cmd /c "`"$vcvars`" >nul 2>&1 && $Command"
    if ($LASTEXITCODE -ne 0) { throw "command failed (exit $LASTEXITCODE): $Command" }
}

# ---- fly -------------------------------------------------------------------
if ($Rtls) {
    # The whole flight on one clock: our own ascent and boostback down to the
    # entry interface at 65 km, the donor core from there to the deck.
    $blsBuild = Join-Path $root "build-bls"
    if (-not (Test-Path (Join-Path $blsBuild "build.ninja"))) {
        Write-Host "configuring the donor core..." -ForegroundColor Cyan
        Invoke-InMsvcEnv "`"$cmake`" -S `"$root\external\booster-lander-simulator`" -B `"$blsBuild`" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBL_CUDA=OFF -DCMAKE_MAKE_PROGRAM=`"$ninja`""
    }
    Write-Host "building the donor core..." -ForegroundColor Cyan
    Invoke-InMsvcEnv "`"$cmake`" --build `"$blsBuild`""

    if ($Out -eq "out") { $Out = "out_rtls" }
    # NOT $rtls: PowerShell variables are case-insensitive, so that name is the
    # -Rtls switch parameter, and assigning an array to a [switch]-typed
    # variable fails the cast at the call site with a message that points at the
    # command line rather than at this line.
    $rtlsArgs = @("--out", $Out, "--seed", $Seed, "--aim-passes", $AimPasses)
    if ($Gamma -ne 0.0) { $rtlsArgs += @("--gamma", $Gamma) }
    Write-Host "`nflying the solo RTLS profile: pad to pad..." -ForegroundColor Cyan
    & $python (Join-Path $root "bridge\rtls_flight.py") @rtlsArgs
    if ($LASTEXITCODE -ne 0) { throw "rtls_flight failed (exit $LASTEXITCODE)" }
}
elseif ($Scenario) {
    # Donor path: the full return from stage separation, with an atmosphere.
    # pdg models thrust and gravity only, so it cannot fly this profile.
    $blsBuild = Join-Path $root "build-bls"
    if (-not (Test-Path (Join-Path $blsBuild "build.ninja"))) {
        Write-Host "configuring the donor core..." -ForegroundColor Cyan
        Invoke-InMsvcEnv "`"$cmake`" -S `"$root\external\booster-lander-simulator`" -B `"$blsBuild`" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBL_CUDA=OFF -DCMAKE_MAKE_PROGRAM=`"$ninja`""
    }
    Write-Host "building the donor core..." -ForegroundColor Cyan
    Invoke-InMsvcEnv "`"$cmake`" --build `"$blsBuild`""

    if ($Out -eq "out") { $Out = "out_$Scenario" }
    Write-Host "`nflying the $Scenario profile..." -ForegroundColor Cyan
    & $python (Join-Path $root "bridge\bls_flight.py") --scenario $Scenario --seed $Seed --out $Out
    if ($LASTEXITCODE -ne 0) { throw "bls_flight failed (exit $LASTEXITCODE)" }
}
else {
    # ---- configure + build the guidance core -------------------------------
    $buildDir = Join-Path $root "build"
    if (-not (Test-Path (Join-Path $buildDir "build.ninja"))) {
        Write-Host "configuring..." -ForegroundColor Cyan
        Invoke-InMsvcEnv "`"$cmake`" -S `"$root`" -B `"$buildDir`" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_MAKE_PROGRAM=`"$ninja`""
    }
    Write-Host "building..." -ForegroundColor Cyan
    Invoke-InMsvcEnv "`"$cmake`" --build `"$buildDir`""

    if ($Build) {
        Write-Host "running unit tests..." -ForegroundColor Cyan
        & $ctest --test-dir $buildDir --output-on-failure
        exit $LASTEXITCODE
    }

    $flight = @("--out", $Out, "--quiet")
    if ($Preset -ne "default") {
        $flight += @("--preset", $Preset)
        if ($Out -eq "out") { $Out = "out_$Preset"; $flight[1] = $Out }
    }
    if ($R0) { $flight += @("--r0", $R0) }
    if ($V0) { $flight += @("--v0", $V0) }
    if ($Nodes -gt 0) { $flight += @("--nodes", $Nodes) }
    if ($TfGuess -gt 0.0) { $flight += @("--tf-guess", $TfGuess, "--tf-max", [math]::Max(3.0 * $TfGuess, 120.0)) }
    if ($ThrustScale -ne 1.0) { $flight += @("--thrust-scale", $ThrustScale) }
    if ($MisalignDeg -ne 0.0) { $flight += @("--misalign-deg", $MisalignDeg) }
    if ($ActuatorTau -ne 0.0) { $flight += @("--actuator-tau", $ActuatorTau) }

    Write-Host "`nsolving guidance and flying the descent..." -ForegroundColor Cyan
    & (Join-Path $buildDir "pdg_flight.exe") @flight
    if ($LASTEXITCODE -ne 0) { throw "pdg_flight failed (exit $LASTEXITCODE)" }
}

# ---- replay ----------------------------------------------------------------
# matplotlib renders on the CPU and is the default: Rerun needs a working GPU,
# and on a virtual adapter its window comes up transparent.
if ($Renderer -eq "rerun") {
    $viz = @((Join-Path $root "viz\land2d.py"), "--data", $Out, "--speed", $Speed,
             "--vehicle-scale", $VehicleScale)
    if ($NoViewer) { $viz += "--no-spawn" }
    if ($Save) { $viz += @("--save", $Save) }
    Write-Host "`nstreaming to Rerun at ${Speed}x..." -ForegroundColor Cyan
} else {
    $viz = @((Join-Path $root "viz\land2d_mpl.py"), "--data", $Out, "--speed", $Speed,
             "--vehicle-scale", $VehicleScale)
    if ($Follow -gt 0.0) { $viz += @("--follow", $Follow) }
    else { $viz += @("--zoom-at", $ZoomAt, "--zoom-to", $ZoomTo,
                     "--zoom-pause", $ZoomPause) }
    if ($Save) { $viz += @("--save", $Save) }
    if ($NoViewer -and -not $Save) {
        Write-Host "-NoViewer only applies to the Rerun renderer; showing the window." `
            -ForegroundColor Yellow
    }
    Write-Host "`nreplaying with matplotlib at ${Speed}x..." -ForegroundColor Cyan
}

& $python @viz
exit $LASTEXITCODE
