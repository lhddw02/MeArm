# Respect an active Conda/venv before considering the bundled Python fallback.
param([switch]$Demo)
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$bundledPython = Join-Path $env:USERPROFILE '.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe'
$activePrefix = if ($env:CONDA_PREFIX) { $env:CONDA_PREFIX } elseif ($env:VIRTUAL_ENV) { $env:VIRTUAL_ENV } else { $null }
if ($activePrefix) {
    $taskPython = Join-Path $activePrefix 'python.exe'
    if (-not (Test-Path -LiteralPath $taskPython)) {
        $taskPython = Join-Path $activePrefix 'Scripts\python.exe'
    }
    if (-not (Test-Path -LiteralPath $taskPython)) {
        throw "No Python executable found in the active environment: $activePrefix"
    }
} elseif (Test-Path -LiteralPath $bundledPython) {
    $taskPython = $bundledPython
} else {
    $taskPython = 'python'
}
$taskArguments = @((Join-Path $taskRoot 'scripts\run_mujoco.py'))
if ($Demo) { $taskArguments += '--demo' }
& $taskPython @taskArguments
exit $LASTEXITCODE
