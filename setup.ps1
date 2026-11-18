<#
.SYNOPSIS
    One-time setup: clone the upstream repositories and create the virtualenv.

    Installing the MSVC C++ toolchain is NOT done here -- it is a multi-gigabyte
    system-wide install. If `run.ps1` cannot find one, install it with:

        winget install --id Microsoft.VisualStudio.2022.BuildTools -e `
          --override "--quiet --wait --norestart --nocache `
                      --add Microsoft.VisualStudio.Workload.VCTools `
                      --add Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
                      --add Microsoft.VisualStudio.Component.Windows11SDK.22621"
#>
[CmdletBinding()]
param([switch]$SkipClone)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot
$external = Join-Path $root "external"

# Pinned to the commits this repository was built and checked against: the
# donor patch is a diff against exactly that revision, and upstream HEAD moves.
$repos = [ordered]@{
    "pdg" = @{
        url = "https://github.com/Krataios14/pdg.git"
        rev = "a14a24a96d8974da2aaaa36bda25e7d30bd0a6c5" }
    "SCvx" = @{
        url = "https://github.com/EmbersArc/SCvx.git"
        rev = "6476c392d5f2d7498d4592c14417618ee828966d" }
    "RL_Rocket_6DOF" = @{
        url = "https://github.com/Tuxliri/RL_Rocket_6DOF.git"
        rev = "7d68e2aff82dd17f069268a5b4c972f816f907ab" }
    "booster-lander-simulator" = @{
        url = "https://github.com/bochen2029-pixel/booster-lander-simulator.git"
        rev = "99c7f6582af002c130e9ecd5c6e546b1f144d034" }
}

if (-not $SkipClone) {
    New-Item -ItemType Directory -Force $external | Out-Null
    foreach ($name in $repos.Keys) {
        $dest = Join-Path $external $name
        if (Test-Path $dest) {
            Write-Host "$name already present, skipping" -ForegroundColor DarkGray
        } else {
            Write-Host "fetching $name @ $($repos[$name].rev.Substring(0, 7))..." -ForegroundColor Cyan
            # A shallow fetch of one commit by hash; GitHub serves these.
            git init --quiet $dest
            git -C $dest remote add origin $repos[$name].url
            git -C $dest fetch --quiet --depth 1 origin $repos[$name].rev
            if ($LASTEXITCODE -ne 0) { throw "git fetch failed for $name" }
            git -C $dest checkout --quiet FETCH_HEAD
            if ($LASTEXITCODE -ne 0) { throw "git checkout failed for $name" }
        }

        # patches/<name>.patch, if present, is applied once.
        $patch = Join-Path $root "patches\$name.patch"
        if (Test-Path $patch) {
            # Windows PowerShell turns redirected native stderr into a
            # terminating error under "Stop", so relax it for the probe.
            $ErrorActionPreference = "Continue"
            git -C $dest apply --check --reverse $patch 2>$null
            $applied = ($LASTEXITCODE -eq 0)
            $ErrorActionPreference = "Stop"
            if ($applied) {
                Write-Host "  $name patch already applied" -ForegroundColor DarkGray
            } else {
                git -C $dest apply $patch
                if ($LASTEXITCODE -ne 0) { throw "patch did not apply to $name" }
                Write-Host "  applied patches\$name.patch" -ForegroundColor Cyan
            }
        }
    }
}

$venv = Join-Path $root ".venv"
if (-not (Test-Path $venv)) {
    Write-Host "creating virtualenv..." -ForegroundColor Cyan
    python -m venv $venv
}
$python = Join-Path $venv "Scripts\python.exe"

Write-Host "installing Python dependencies..." -ForegroundColor Cyan
& $python -m pip install --upgrade pip
& $python -m pip install -r (Join-Path $root "requirements.txt")
if ($LASTEXITCODE -ne 0) { throw "pip install failed" }

Write-Host "`nsetup complete. Next:  .\run.ps1" -ForegroundColor Green
