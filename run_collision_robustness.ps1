# Frozen evaluation schedule. Existing completed runs are reused on resume.
$ErrorActionPreference = 'Stop'
$python = 'D:\detection\robot_human_isaac6\runtime\python.bat'
$scenes = @('headon_collision', 'perpendicular_collision', 'diagonal_collision',
    'cutin_collision', 'rear_end_collision', 'perpendicular_nearmiss',
    'diagonal_nearmiss', 'cutin_nearmiss', 'multi_person_collision')
$hard = @('perpendicular_nearmiss_hard', 'diagonal_nearmiss_hard', 'cutin_nearmiss_hard')
$drop = @('headon_collision', 'cutin_collision', 'rear_end_collision', 'perpendicular_nearmiss_hard')
$root = Join-Path $PSScriptRoot 'outputs\third_person_collision_prediction_robustness'

function Invoke-Run([string]$phase, [string]$scene, [int]$seed, [double]$rate) {
    $folder = Join-Path $root (Join-Path $phase (Join-Path $scene "seed_$seed"))
    if ($phase -eq 'dropout') {
        $percent = [int][Math]::Round($rate * 100)
        $folder = Join-Path $folder ('rate_{0:d2}_dropseed_101' -f $percent)
    }
    if (Test-Path -LiteralPath (Join-Path $folder 'summary.json')) {
        Write-Output "REUSE $phase $scene seed=$seed rate=$rate"
        return
    }
    $args = @('third_person_collision_prediction.py', '--scenario', $scene,
        '--seed', "$seed", '--seconds', '35', '--detector-model', 'yolo26n.pt',
        '--optimized', '--robustness-phase', $phase, '--headless')
    if ($phase -eq 'dropout') {
        $args += @('--detection-dropout-mode', 'random',
            '--detection-dropout-rate', "$rate", '--dropout-seed', '101')
    }
    Write-Output "START $phase $scene seed=$seed rate=$rate"
    & $python @args 2>&1 | Select-String 'THIRD_PERSON_RUN|PROGRESS|RESULT|Traceback|Error|Exception|Shutting Down'
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath (Join-Path $folder 'summary.json'))) {
        throw "Run failed: $phase $scene seed=$seed rate=$rate"
    }
}

foreach ($scene in $scenes) {
    foreach ($seed in @(23, 31)) {
        Invoke-Run 'multiseed' $scene $seed 0
    }
}
foreach ($scene in $hard) {
    Invoke-Run 'hard_nearmiss' $scene 17 0
}
foreach ($scene in $drop) {
    foreach ($rate in @(0.1, 0.2, 0.3)) {
        Invoke-Run 'dropout' $scene 17 $rate
    }
}
Write-Output 'ALL_ROBUSTNESS_RUNS_FINISHED'
