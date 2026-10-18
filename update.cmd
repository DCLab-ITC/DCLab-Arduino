@echo off
rem Updates this library to the latest version on GitHub (Windows).
rem Double-click it in the library's folder (Documents\Arduino\libraries\DCLab).
rem The PowerShell below the #PS# line does the work; one cmd line runs it and
rem ends, so replacing this file while it runs is harmless.
set "DCLAB_SELF=%~f0"
powershell -NoProfile -ExecutionPolicy Bypass -Command "iex ((Get-Content -Raw $env:DCLAB_SELF) -split '#PS#\r?\n')[1]" & pause & exit /b
#PS#
$ErrorActionPreference = 'Stop'
$repo = 'https://github.com/DCLab-ITC/DCLab-Arduino'
$here = Split-Path -Parent $env:DCLAB_SELF
function Ver($dir) {
  (Get-Content "$dir\library.properties" | Where-Object { $_ -like 'version=*' }) -replace '^version=', ''
}
$before = Ver $here
if (Test-Path "$here\.git") {
  if (git -C $here status --porcelain --untracked-files=no) {
    throw "You changed files in $here (git status): commit or undo them, then run this again."
  }
  git -C $here pull --ff-only $repo main
  if ($LASTEXITCODE) { throw 'git pull failed' }
} else {
  [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
  $work = Join-Path $env:TEMP ('dclab-update-' + [guid]::NewGuid())
  New-Item -ItemType Directory $work | Out-Null
  try {
    Write-Host "Downloading $repo ..."
    Invoke-WebRequest -UseBasicParsing "$repo/archive/refs/heads/main.zip" -OutFile "$work\new.zip"
    Expand-Archive "$work\new.zip" "$work\new"
    # one top folder: DCLab-Arduino-main
    $new = (Get-ChildItem -Directory "$work\new" | Select-Object -First 1).FullName
    if (-not (Select-String -Quiet -Path "$new\library.properties" -Pattern '^name=DCLab$')) {
      throw 'The download is not the DCLab library.'
    }
    Get-ChildItem -Force $here | Remove-Item -Recurse -Force
    Copy-Item -Recurse -Force "$new\*" $here
  } finally {
    Remove-Item -Recurse -Force $work -ErrorAction SilentlyContinue
  }
}
Write-Host "DCLab library $before -> $(Ver $here)"
Write-Host 'Done. Restart the Arduino IDE if it is open.'
