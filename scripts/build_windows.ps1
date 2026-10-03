param(
    [string]$VcRoot = 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.44.35207',
    [string]$SdkRoot = 'C:\Program Files (x86)\Windows Kits\10',
    [string]$SdkVersion = '10.0.26100.0'
)
$ErrorActionPreference = 'Stop'
$taskProject = Split-Path $PSScriptRoot -Parent
$taskOutput = Join-Path $taskProject 'build'
New-Item -ItemType Directory -Path $taskOutput -Force | Out-Null
$env:INCLUDE = "$VcRoot\include;$SdkRoot\Include\$SdkVersion\ucrt;$SdkRoot\Include\$SdkVersion\shared;$SdkRoot\Include\$SdkVersion\um"
$env:LIB = "$VcRoot\lib\x64;$SdkRoot\Lib\$SdkVersion\ucrt\x64;$SdkRoot\Lib\$SdkVersion\um\x64"
$env:PATH = "$VcRoot\bin\Hostx64\x64;" + $env:PATH
Push-Location $taskProject
try {
    foreach ($taskTarget in @('scenario_eval','experiment','test')) {
        $taskSource = if ($taskTarget -eq 'test') { 'tests/test.cpp' } else { "src/$taskTarget.cpp" }
        & "$VcRoot\bin\Hostx64\x64\cl.exe" /nologo /std:c++20 /O2 /EHsc /W4 /Iinclude $taskSource "/Febuild/$taskTarget.exe" "/Fobuild/$taskTarget.obj"
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $taskTarget" }
    }
    & '.\build\test.exe'
    if ($LASTEXITCODE -ne 0) { throw 'Geometry validation failed' }
    & '.\build\experiment.exe' --scenario
    if ($LASTEXITCODE -ne 0) { throw 'Scenario demo failed' }
} finally {
    Pop-Location
}
