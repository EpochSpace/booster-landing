<#
.SYNOPSIS
    Build model-ru.pdf from model-ru.tex (Russian-language full model).

    Same MiKTeX lookup as build.ps1: a machine-scope install only reaches PATH
    in shells started after it, so the compiler is found by path.

    Cyrillic needs the T2A encoding. Glyphs come from cm-super (Type 1); the
    first build may run METAFONT to generate the LH font *metrics*, which takes
    a minute and is cached afterwards. The PDF itself contains no bitmap fonts —
    check with `pdffonts model-ru.pdf` if you want to confirm.
#>
[CmdletBinding()]
param([switch]$Clean)

$ErrorActionPreference = "Stop"
$here = $PSScriptRoot

if ($Clean) {
    Get-ChildItem $here -Include model-ru.aux, model-ru.log, model-ru.out, model-ru.toc `
        -File -Recurse | Remove-Item -Force
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
    throw "pdflatex not found. Install MiKTeX: winget install --id MiKTeX.MiKTeX -e"
}

Push-Location $here
try {
    # Two passes: the first writes the cross-references and the table of
    # contents, the second resolves them.
    #
    # ErrorActionPreference has to be relaxed across the call. MiKTeX prints
    # "no MiKTeX administrator has checked for updates" on stderr on every run,
    # and under `Stop` PowerShell 5.1 turns any native stderr output into a
    # terminating NativeCommandError -- so the build fails on a notice, with an
    # error that points at this line rather than at anything in the document.
    # The exit code is the real signal, so that is what gets checked.
    foreach ($pass in 1, 2) {
        Write-Host "pass $pass..." -ForegroundColor Cyan
        $saved = $ErrorActionPreference
        $ErrorActionPreference = "Continue"
        # 2>$null discards MiKTeX's update notice; the exit code is the signal
        $out = & $pdflatex -interaction=nonstopmode model-ru.tex 2>$null
        $code = $LASTEXITCODE
        $ErrorActionPreference = $saved
        $errors = $out | Select-String -Pattern "^!|LaTeX Error|Undefined control sequence"
        if ($errors -or $code -ne 0) {
            $errors | Select-Object -First 15 | ForEach-Object { Write-Host $_ -ForegroundColor Red }
            throw "LaTeX reported errors (exit $code); see model-ru.log"
        }
    }
    $unresolved = $out | Select-String -Pattern "LaTeX Warning: Reference"
    if ($unresolved) {
        Write-Host "warning: unresolved references remain" -ForegroundColor Yellow
    }
    $written = $out | Select-String -Pattern "Output written"
    Write-Host $written -ForegroundColor Green
}
finally {
    Pop-Location
}
