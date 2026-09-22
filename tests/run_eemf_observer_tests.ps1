$ErrorActionPreference = 'Stop'

$compiler = Get-Command clang -ErrorAction SilentlyContinue
if ($null -eq $compiler) {
    $compiler = Get-Command gcc -ErrorAction SilentlyContinue
}
if ($null -eq $compiler) {
    throw 'Host clang or gcc is required to run EEMF observer tests.'
}

$outputDirectory = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force $outputDirectory | Out-Null
$executable = Join-Path $outputDirectory 'test_eemf_observer.exe'

& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    '-ICore/Config' '-ICore/Control' '-ICore/Algorithm' '-ICore/Common' `
    'tests/test_eemf_observer.c' 'Core/Control/eemf_observer.c' `
    'Core/Control/pll.c' 'Core/Algorithm/pi_controller.c' `
    'Core/Algorithm/transform.c' 'Core/Config/sensorless_config.c' `
    '-lm' '-o' $executable
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

& $executable
