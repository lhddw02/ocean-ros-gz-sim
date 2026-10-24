# Read-only readiness check for Windows, WSL2, Docker Desktop, and NVIDIA.
# Run in PowerShell: .\scripts\check-windows.ps1
# This script does not install software, start Docker, or change settings.
[CmdletBinding()]
param()

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

function Convert-NativeOutput {
    param([byte[]] $Bytes)
    if ($Bytes.Length -eq 0) { return '' }
    $sampleLength = [Math]::Min($Bytes.Length, 512)
    $nullCount = 0
    for ($i = 0; $i -lt $sampleLength; $i++) {
        if ($Bytes[$i] -eq 0) { $nullCount++ }
    }
    # wsl.exe commonly emits UTF-16LE when its output is redirected.
    if (($Bytes.Length -ge 2 -and $Bytes[0] -eq 255 -and $Bytes[1] -eq 254) -or
        $nullCount -gt ($sampleLength / 10)) {
        return [Text.Encoding]::Unicode.GetString($Bytes).Trim([char]0xFEFF).Trim()
    }
    return [Text.Encoding]::UTF8.GetString($Bytes).Trim([char]0xFEFF).Trim()
}

function Invoke-ReadOnlyCommand {
    param([string] $FilePath, [string] $Arguments, [int] $TimeoutSeconds = 15)
    $process = New-Object Diagnostics.Process
    $stdout = New-Object IO.MemoryStream
    $stderr = New-Object IO.MemoryStream
    try {
        $process.StartInfo.FileName = $FilePath
        $process.StartInfo.Arguments = $Arguments
        $process.StartInfo.UseShellExecute = $false
        $process.StartInfo.CreateNoWindow = $true
        $process.StartInfo.RedirectStandardOutput = $true
        $process.StartInfo.RedirectStandardError = $true
        [void] $process.Start()
        # Drain both pipes concurrently to avoid blocking a verbose command.
        $stdoutTask = $process.StandardOutput.BaseStream.CopyToAsync($stdout)
        $stderrTask = $process.StandardError.BaseStream.CopyToAsync($stderr)
        if (-not $process.WaitForExit($TimeoutSeconds * 1000)) {
            try { $process.Kill() } catch { }
            [void] $process.WaitForExit(2000)
            try { [void] $stdoutTask.Wait(2000); [void] $stderrTask.Wait(2000) } catch { }
            return [pscustomobject]@{ ExitCode = -1; TimedOut = $true; Output = ''; Error = 'Command timed out.' }
        }
        [void] $stdoutTask.GetAwaiter().GetResult()
        [void] $stderrTask.GetAwaiter().GetResult()
        return [pscustomobject]@{
            ExitCode = $process.ExitCode
            TimedOut = $false
            Output = Convert-NativeOutput $stdout.ToArray()
            Error = Convert-NativeOutput $stderr.ToArray()
        }
    } catch {
        return [pscustomobject]@{ ExitCode = -1; TimedOut = $false; Output = ''; Error = $_.Exception.Message }
    } finally {
        $process.Dispose()
        $stdout.Dispose()
        $stderr.Dispose()
    }
}

function Find-Application {
    param([string] $Name, [string[]] $Candidates)
    $command = Get-Command $Name -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) { return $command.Source }
    foreach ($candidate in $Candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) { return $candidate }
    }
    return $null
}

Write-Host 'Ocean simulation: Windows readiness check' -ForegroundColor Cyan
Write-Host 'Read-only checks; no software or settings will be changed.'
Write-Host ''
$nextSteps = New-Object 'System.Collections.Generic.List[string]'

try {
    $osInfo = Get-CimInstance Win32_OperatingSystem
    $computerInfo = Get-CimInstance Win32_ComputerSystem
    $ramGiB = [Math]::Round($computerInfo.TotalPhysicalMemory / 1GB, 1)
    Write-Host ('Windows: {0}, build {1}' -f $osInfo.Caption, $osInfo.BuildNumber)
    Write-Host ('Resources: {0} GiB RAM, {1} logical processors' -f $ramGiB, $computerInfo.NumberOfLogicalProcessors)
} catch {
    Write-Host 'Windows resources: unavailable in this restricted session.' -ForegroundColor Yellow
}

$dockerInstall = $null
try {
    $dockerInstall = Get-ItemProperty -Path `
        'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*', `
        'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*', `
        'HKLM:\Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*' `
        -ErrorAction SilentlyContinue |
        Where-Object { $_.PSObject.Properties['DisplayName'] -and $_.DisplayName -eq 'Docker Desktop' } |
        Select-Object -First 1
} catch { }
if ($dockerInstall) {
    Write-Host ('Docker Desktop: installed, version {0}' -f $dockerInstall.DisplayVersion)
} else {
    Write-Host 'Docker Desktop: installation registration not found.' -ForegroundColor Yellow
}

$dockerCandidates = @(
    (Join-Path $env:ProgramFiles 'Docker\Docker\resources\bin\docker.exe'),
    (Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\resources\bin\docker.exe'),
    (Join-Path $env:LOCALAPPDATA 'Programs\Docker\Docker\resources\bin\docker.exe'),
    (Join-Path $env:LOCALAPPDATA 'Docker\resources\bin\docker.exe')
)
if ($dockerInstall -and $dockerInstall.PSObject.Properties['InstallLocation'] -and $dockerInstall.InstallLocation) {
    $dockerCandidates += Join-Path $dockerInstall.InstallLocation 'resources\bin\docker.exe'
}
$docker = Find-Application 'docker.exe' $dockerCandidates
if ($docker) {
    $dockerVersion = Invoke-ReadOnlyCommand $docker '--version'
    if ($dockerVersion.ExitCode -eq 0) {
        Write-Host ('Docker CLI: {0}' -f $dockerVersion.Output)
    } else {
        Write-Host 'Docker CLI: installed, but the command could not run in this session.' -ForegroundColor Yellow
    }
    if (-not (Get-Command docker.exe -CommandType Application -ErrorAction SilentlyContinue)) {
        Write-Host 'Docker CLI is installed but is absent from this terminal PATH.' -ForegroundColor Yellow
        $nextSteps.Add('Close and reopen PowerShell or Windows Terminal after installing Docker to refresh PATH.')
    }
    $composeVersion = Invoke-ReadOnlyCommand $docker 'compose version --short'
    if ($composeVersion.ExitCode -eq 0) {
        Write-Host ('Docker Compose: {0}' -f $composeVersion.Output)
    } else {
        Write-Host 'Docker Compose: unavailable.' -ForegroundColor Yellow
        $nextSteps.Add('Ensure Docker Desktop includes the Compose plugin, then rerun this check.')
    }
    $engine = Invoke-ReadOnlyCommand $docker 'info --format "{{.OSType}}|{{.ServerVersion}}"'
    if ($engine.ExitCode -eq 0) {
        $engineParts = $engine.Output.Split('|')
        Write-Host ('Docker engine: reachable ({0})' -f $engine.Output.Replace('|', ', version ')) -ForegroundColor Green
        if ($engineParts[0] -ne 'linux') {
            $nextSteps.Add('Switch Docker Desktop to Linux containers and enable the WSL2 engine.')
        }
    } else {
        $engineStatus = if ($engine.TimedOut) { 'check timed out' } else { 'not reachable' }
        Write-Host ('Docker engine: {0}.' -f $engineStatus) -ForegroundColor Yellow
        $nextSteps.Add('After WSL2 setup, open Docker Desktop and wait until its engine is running; enable the WSL2 engine.')
    }
} else {
    Write-Host 'Docker CLI: not found in PATH or standard Docker Desktop locations.' -ForegroundColor Yellow
    $nextSteps.Add('Install Docker Desktop for Windows, or verify its installation location, then reopen your terminal.')
}

$wsl = Find-Application 'wsl.exe' @(
    (Join-Path $env:WINDIR 'System32\wsl.exe'),
    (Join-Path $env:WINDIR 'Sysnative\wsl.exe')
)
if ($wsl) {
    $wslVersion = Invoke-ReadOnlyCommand $wsl '--version'
    Write-Host ''
    Write-Host 'WSL version:'
    if ($wslVersion.Output) { Write-Host $wslVersion.Output }
    if ($wslVersion.Error) { Write-Host $wslVersion.Error -ForegroundColor Yellow }
    if ($wslVersion.TimedOut) { Write-Host 'Version check timed out.' -ForegroundColor Yellow }
    $wslList = Invoke-ReadOnlyCommand $wsl '--list --verbose'
    Write-Host 'WSL distributions:'
    if ($wslList.Output) { Write-Host $wslList.Output }
    if ($wslList.Error) { Write-Host $wslList.Error -ForegroundColor Yellow }
    if ($wslList.ExitCode -ne 0) {
        $nextSteps.Add('If WSL reports that it is not installed, run wsl --install -d Ubuntu-22.04 in an Administrator PowerShell, then restart Windows if requested.')
    } elseif ($wslList.Output -notmatch 'Ubuntu') {
        $nextSteps.Add('Install an Ubuntu 22.04 WSL distribution: wsl --install -d Ubuntu-22.04; complete its first-run username/password setup.')
    } else {
        $nextSteps.Add('Confirm your Ubuntu distribution shows VERSION 2, then enable it in Docker Desktop > Settings > Resources > WSL Integration.')
    }
    if ($wslVersion.ExitCode -ne 0 -and $wslList.ExitCode -eq 0) {
        $nextSteps.Add('Update WSL with wsl --update so WSLg and the GPU support are current.')
    }
} else {
    Write-Host 'WSL: executable unavailable.' -ForegroundColor Yellow
    $nextSteps.Add('Install WSL2 and Ubuntu 22.04 from an Administrator PowerShell: wsl --install -d Ubuntu-22.04.')
}

$nvidia = Find-Application 'nvidia-smi.exe' @(
    (Join-Path $env:WINDIR 'System32\nvidia-smi.exe'),
    (Join-Path $env:ProgramFiles 'NVIDIA Corporation\NVSMI\nvidia-smi.exe')
)
Write-Host ''
if ($nvidia) {
    $gpu = Invoke-ReadOnlyCommand $nvidia '--query-gpu=name,driver_version,memory.total --format=csv,noheader'
    if ($gpu.ExitCode -eq 0) {
        Write-Host ('NVIDIA GPU (model, Windows driver, VRAM): {0}' -f $gpu.Output) -ForegroundColor Green
        Write-Host 'Container GPU access and Gazebo graphics still require the WSL project checks.'
    } else {
        Write-Host 'NVIDIA GPU: nvidia-smi failed.' -ForegroundColor Yellow
        $nextSteps.Add('Check the NVIDIA Windows driver and rerun nvidia-smi; WSL uses the Windows driver.')
    }
} else {
    Write-Host 'NVIDIA GPU: nvidia-smi not found.' -ForegroundColor Yellow
    $nextSteps.Add('Install or update the NVIDIA Windows graphics driver, then rerun this check.')
}

Write-Host ''
Write-Host 'Next steps:' -ForegroundColor Cyan
foreach ($step in ($nextSteps | Select-Object -Unique)) { Write-Host ('- {0}' -f $step) }
Write-Host '- In Ubuntu, follow README.md: build, check, smoke, then start the ocean simulation.'
Write-Host '- GPU detection here does not confirm accelerated Gazebo rendering; check must report the RTX 3060 OpenGL renderer.'
