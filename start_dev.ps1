param(
    [string]$Profile = "home_windows",
    [string]$Action = "up"
)

if ($Profile -eq "home_windows") {
    docker compose -f compose.yaml --profile home_windows $Action em_robot_home_windows
    exit $LASTEXITCODE
}

if ($Profile -eq "work_ubuntu") {
    docker compose -f compose.yaml -f compose.ubuntu.yaml --profile work_ubuntu $Action em_robot_work_ubuntu
    exit $LASTEXITCODE
}

Write-Error "Unsupported profile '$Profile'. Use home_windows or work_ubuntu."
exit 1
