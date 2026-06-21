$ScriptDir = $PSScriptRoot
$LogFile   = "$ScriptDir\build-log.txt"

# Tee all output to a log file so you can read it even if the window closes
Start-Transcript -Path $LogFile -Force

try {

$ErrorActionPreference = "Stop"

$OBS_INSTALL  = "C:\Program Files\obs-studio"
$OBS_DLL      = "$OBS_INSTALL\bin\64bit\obs.dll"
$OBS_VERSION  = "31.0.0"
$SDK_DIR      = "$ScriptDir\obs-sdk"
$VSWHERE      = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"


# Find cmake.exe explicitly (avoids PATH issues when running as admin)
$cmakeCmd = Get-Command cmake -ErrorAction SilentlyContinue
$cmakeExe = if ($cmakeCmd) { $cmakeCmd.Source } else { $null }
if (-not $cmakeExe) {
    $cmakeExe = "C:\Program Files\CMake\bin\cmake.exe"
}
if (-not (Test-Path $cmakeExe)) { throw "cmake.exe not found. Is CMake installed?" }
Write-Host "   cmake: $cmakeExe"

function Step($n, $msg) { Write-Host "`n[$n/5] $msg" -ForegroundColor Cyan }
function OK($msg)        { Write-Host "   OK: $msg"  -ForegroundColor Green }
function Info($msg)      { Write-Host "   $msg" }

Write-Host "============================================" -ForegroundColor White
Write-Host " Dynamic Autocrop - Build and Install"      -ForegroundColor White
Write-Host "============================================" -ForegroundColor White
Write-Host " Log file: $LogFile"

#  Step 1: Find / install Visual Studio 
Step 1 "Checking for Visual Studio C++ tools..."
Write-Host "   vswhere path: $VSWHERE"
Write-Host "   vswhere exists: $(Test-Path $VSWHERE)"

$vsPath = $null
if (Test-Path $VSWHERE) {
    $vsPath = & $VSWHERE -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    Write-Host "   vswhere result: '$vsPath'"
}

if (-not $vsPath) {
    Info "Not found. Downloading Visual Studio Build Tools (free, ~4 GB)..."
    $installer = "$env:TEMP\vs_buildtools.exe"
    Invoke-WebRequest "https://aka.ms/vs/17/release/vs_buildtools.exe" -OutFile $installer
    $proc = Start-Process $installer -ArgumentList @(
        "--passive","--wait","--norestart",
        "--add","Microsoft.VisualStudio.Workload.VCTools",
        "--add","Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
        "--add","Microsoft.VisualStudio.Component.Windows11SDK.22621"
    ) -Wait -PassThru
    if ($proc.ExitCode -ne 0) { throw "VS installer failed (exit code $($proc.ExitCode))" }
    $vsPath = & $VSWHERE -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2>$null
    if (-not $vsPath) { throw "VS installed but vswhere still finds nothing. Restart and try again." }
}
OK "VS at: $vsPath"

# Find dumpbin + lib directly (no vcvars needed)
$msvcToolsBase = "$vsPath\VC\Tools\MSVC"
Write-Host "   MSVC tools base: $msvcToolsBase"
Write-Host "   Exists: $(Test-Path $msvcToolsBase)"

$msvcVersion = (Get-ChildItem $msvcToolsBase | Sort-Object Name -Descending | Select-Object -First 1).Name
Write-Host "   MSVC version: $msvcVersion"

$msvcBin = "$msvcToolsBase\$msvcVersion\bin\Hostx64\x64"
$dumpbin = "$msvcBin\dumpbin.exe"
$libExe  = "$msvcBin\lib.exe"
Write-Host "   dumpbin: $dumpbin  exists=$(Test-Path $dumpbin)"
Write-Host "   lib.exe: $libExe   exists=$(Test-Path $libExe)"

if (-not (Test-Path $dumpbin)) { throw "dumpbin.exe not found at $dumpbin" }
OK "MSVC tools found."

#  Step 2: Download OBS source (headers) 
Step 2 "Checking for OBS development headers..."

if (Test-Path "$SDK_DIR\libobs\obs-module.h") {
    # Even if headers exist, ensure obs-config.h is present
    if (-not (Test-Path "$SDK_DIR\libobs\obs-config.h")) {
        $obsConfigH = "#pragma once`n`n#define LIBOBS_API_MAJOR_VER  31`n#define LIBOBS_API_MINOR_VER  0`n#define LIBOBS_API_PATCH_VER  0`n#define LIBOBS_API_VER ((LIBOBS_API_MAJOR_VER << 24) | (LIBOBS_API_MINOR_VER << 16) | LIBOBS_API_PATCH_VER)"
        $obsConfigH | Set-Content "$SDK_DIR\libobs\obs-config.h" -Encoding Ascii
        Info "Generated missing obs-config.h"
    }
    OK "Already downloaded, skipping."
} else {
    Info "Downloading OBS $OBS_VERSION source (~70 MB)..."
    $obsZip = "$env:TEMP\obs-source.zip"
    $obsTmp = "$env:TEMP\obs-src-extract"
    Invoke-WebRequest "https://github.com/obsproject/obs-studio/archive/refs/tags/$OBS_VERSION.zip" -OutFile $obsZip
    Info "Extracting..."
    if (Test-Path $obsTmp) { Remove-Item $obsTmp -Recurse -Force }
    Expand-Archive $obsZip -DestinationPath $obsTmp -Force
    if (Test-Path $SDK_DIR) { Remove-Item $SDK_DIR -Recurse -Force }
    Move-Item "$obsTmp\obs-studio-$OBS_VERSION" $SDK_DIR
    Remove-Item $obsTmp -Recurse -Force -ErrorAction SilentlyContinue
    Remove-Item $obsZip -Force -ErrorAction SilentlyContinue
    # Generate obs-config.h (normally produced by OBS's own cmake build)
    # Without this file, obs.h fails to parse and all types become undefined.
    $obsConfigH = @"
#pragma once

#define LIBOBS_API_MAJOR_VER  31
#define LIBOBS_API_MINOR_VER  0
#define LIBOBS_API_PATCH_VER  0
#define LIBOBS_API_VER \
    ((LIBOBS_API_MAJOR_VER << 24) | (LIBOBS_API_MINOR_VER << 16) | \
     LIBOBS_API_PATCH_VER)
"@
    $obsConfigH | Set-Content "$SDK_DIR\libobs\obs-config.h" -Encoding Ascii
    Info "Generated obs-config.h"
    OK "Headers ready."
}

#  Step 3: Generate obs.lib from obs.dll 
Step 3 "Generating obs.lib import library..."

if (Test-Path "$SDK_DIR\obs.lib") {
    OK "Already exists, skipping."
} else {
    Write-Host "   OBS DLL path: $OBS_DLL"
    Write-Host "   OBS DLL exists: $(Test-Path $OBS_DLL)"
    if (-not (Test-Path $OBS_DLL)) { throw "Cannot find $OBS_DLL - is OBS installed at $OBS_INSTALL ?" }

    Info "Reading exports from obs.dll..."
    $dumpOutput = & $dumpbin /exports $OBS_DLL
    $exports = $dumpOutput | Where-Object {
        $_ -match '^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]+\s+(\S+)'
    } | ForEach-Object { $Matches[1] }

    if (-not $exports -or $exports.Count -eq 0) { throw "No exports found in obs.dll" }
    Info "Found $($exports.Count) exported symbols."

    $defFile = "$SDK_DIR\obs.def"
    "LIBRARY obs" | Set-Content  $defFile -Encoding Ascii
    "EXPORTS"     | Add-Content  $defFile -Encoding Ascii
    $exports      | Add-Content  $defFile -Encoding Ascii

    & $libExe /def:$defFile /out:"$SDK_DIR\obs.lib" /machine:x64
    if ($LASTEXITCODE -ne 0) { throw "lib.exe failed (exit code $LASTEXITCODE)" }
    OK "obs.lib created."
}

#  Step 4: Configure and build 
Step 4 "Configuring and building..."

$gen = if ($vsPath -match "\\2019\\") { "Visual Studio 16 2019" } else { "Visual Studio 17 2022" }
Info "Generator: $gen"

$buildDir = "$ScriptDir\build"
if (Test-Path $buildDir) { Remove-Item $buildDir -Recurse -Force }

& $cmakeExe -S $ScriptDir -B $buildDir -G $gen -A x64 `
    "-DOBS_INCLUDE_DIR=$SDK_DIR\libobs" `
    "-DOBS_LIB=$SDK_DIR\obs.lib"
if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)" }

& $cmakeExe --build $buildDir --config RelWithDebInfo
if ($LASTEXITCODE -ne 0) { throw "CMake build failed (exit $LASTEXITCODE)" }

#  Step 5: Install 
Step 5 "Installing into OBS..."

& $cmakeExe --install $buildDir --config RelWithDebInfo --prefix $OBS_INSTALL
if ($LASTEXITCODE -ne 0) { throw "cmake install failed (exit $LASTEXITCODE)" }

Write-Host "`n============================================" -ForegroundColor Green
Write-Host " Done! Restart OBS, then:"                    -ForegroundColor Green
Write-Host "  Right-click source > Filters > +"           -ForegroundColor Green
Write-Host "  Dynamic Autocrop"                            -ForegroundColor Green
Write-Host "============================================"  -ForegroundColor Green

} catch {
    Write-Host "`n============================================" -ForegroundColor Red
    Write-Host " BUILD FAILED" -ForegroundColor Red
    Write-Host " Error: $_"   -ForegroundColor Red
    Write-Host "============================================" -ForegroundColor Red
    Write-Host "`n Full log saved to: $LogFile" -ForegroundColor Yellow
}

Stop-Transcript
Read-Host "`nPress Enter to close"
