$ErrorActionPreference = 'Stop'

$compiler = Get-Command clang -ErrorAction SilentlyContinue
if ($null -eq $compiler) {
    $compiler = Get-Command gcc -ErrorAction SilentlyContinue
}
if ($null -eq $compiler) {
    throw 'Host clang or gcc is required to run drive-parameter tests.'
}

$outputDirectory = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $outputDirectory | Out-Null
$executable = Join-Path $outputDirectory 'test_drive_parameters.exe'

& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-ICore/Config' 'tests/test_drive_parameters.c' 'Core/Config/drive_parameters.c' `
    '-o' $executable
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $executable
