param(
    [string]$Profile = "home_windows",
    [string]$Action = "up"
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$composeFile = Join-Path $repoRoot "docker/compose.yaml"
$composeUbuntuFile = Join-Path $repoRoot "docker/compose.ubuntu.yaml"

if ($Profile -eq "home_windows") {
    docker compose -f $composeFile --profile home_windows $Action em_robot_home_windows
    exit $LASTEXITCODE
}

if ($Profile -eq "desktop_replay") {
    docker compose -f $composeFile --profile desktop_replay $Action em_robot_desktop_replay
    exit $LASTEXITCODE
}

if ($Profile -eq "work_ubuntu") {
    docker compose -f $composeFile -f $composeUbuntuFile --profile work_ubuntu $Action em_robot_work_ubuntu
    exit $LASTEXITCODE
}

Write-Error "Unsupported profile '$Profile'. Use home_windows, desktop_replay, or work_ubuntu."
exit 1
