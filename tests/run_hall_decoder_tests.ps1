$ErrorActionPreference = 'Stop'

$compiler = Get-Command clang -ErrorAction SilentlyContinue
if ($null -eq $compiler) {
    $compiler = Get-Command gcc -ErrorAction SilentlyContinue
}
if ($null -eq $compiler) {
    throw 'Host clang or gcc is required to run Hall decoder tests.'
}

$outputDirectory = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $outputDirectory | Out-Null
$executable = Join-Path $outputDirectory 'test_hall_decoder.exe'

& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-ICore/Common' '-ICore/Control' 'tests/test_hall_decoder.c' `
    'Core/Control/hall_decoder.c' '-lm' '-o' $executable
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $executable
