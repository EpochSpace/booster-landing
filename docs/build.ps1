<#
.SYNOPSIS
    Build model.pdf from model.tex.

    A machine-scope MiKTeX install only reaches PATH in shells started after it,
    so this looks the compiler up directly rather than relying on the session.
    LaTeX is run twice: the first pass writes the cross-references, the second
    resolves them.
#>
[CmdletBinding()]
param([switch]$Clean)

$ErrorActionPreference = "Stop"
$here = $PSScriptRoot

if ($Clean) {
    Get-ChildItem $here -Include *.aux, *.log, *.out, *.toc -File -Recurse |
        Remove-Item -Force
    Write-Host "auxiliary files removed" -ForegroundColor Cyan
}

$candidates = @(
    "C:\Program Files\MiKTeX\miktex\bin\x64\pdflatex.exe",
    "$env:LOCALAPPDATA\Programs\MiKTeX\miktex\bin\x64\pdflatex.exe"
)
$pdflatex = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $pdflatex) {
    $onPath = Get-Command pdflatex -ErrorAction SilentlyContinue
    if ($onPath) { $pdflatex = $onPath.Source }
}
if (-not $pdflatex) {
    throw "No pdflatex found. Install one with:  winget install --id MiKTeX.MiKTeX -e"
}

Push-Location $here
try {
    # ErrorActionPreference has to be relaxed across the call: MiKTeX prints
    # "no MiKTeX administrator has checked for updates" on stderr on every run,
    # and under `Stop` PowerShell 5.1 turns native stderr output into a
    # terminating NativeCommandError, failing the build on a notice. The exit
    # code below is the real signal.
    $saved = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    foreach ($pass in 1, 2) {
        Write-Host "pass $pass..." -ForegroundColor Cyan
        & $pdflatex -interaction=nonstopmode model.tex 2>$null | Out-Null
        if ($LASTEXITCODE -ne 0) {
            $ErrorActionPreference = $saved
            & $pdflatex -interaction=nonstopmode model.tex |
                Select-String -Pattern "^!" -Context 0, 3
            throw "pdflatex failed on pass $pass (see model.log)"
        }
    }
    $ErrorActionPreference = $saved
} finally {
    Pop-Location
}

$log = Get-Content (Join-Path $here "model.log") -Raw
$undef = ($log -split "`n" | Select-String "undefined").Count
$pdf = Get-Item (Join-Path $here "model.pdf")
Write-Host ("model.pdf: {0:N0} bytes, {1} undefined references" -f $pdf.Length, $undef) `
    -ForegroundColor Green
