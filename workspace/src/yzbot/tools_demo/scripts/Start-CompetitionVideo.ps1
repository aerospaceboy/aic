param(
    [string]$MappingRules,
    [string]$VmAddress = $env:YZBOT_VM_HOST,
    [string]$VmUser = $env:YZBOT_VM_USER,
    [string]$VmHostKey = $env:YZBOT_VM_HOST_KEY,
    [string]$VmPassword = $env:YZBOT_VM_PASSWORD,
    [switch]$PrepareOnly,
    [switch]$GeneratorOnly,
    [switch]$AutoStart,
    [switch]$CheckOnly
)
$ErrorActionPreference = 'Stop'
$plink = $env:YZBOT_PLINK
$studio = $env:YZBOT_COSTUDIO
foreach ($entry in @{
    YZBOT_VM_HOST = $VmAddress; YZBOT_VM_USER = $VmUser
    YZBOT_VM_HOST_KEY = $VmHostKey; YZBOT_VM_PASSWORD = $VmPassword
    YZBOT_PLINK = $plink
}.GetEnumerator()) {
    if ([string]::IsNullOrWhiteSpace($entry.Value)) {
        throw "Missing private connection configuration: $($entry.Key)"
    }
}
$sshOptions = @('-batch','-pw',$VmPassword,'-hostkey',$VmHostKey,"${VmUser}@${VmAddress}")
$OutputEncoding = [Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$evidence = Join-Path $PSScriptRoot ('video-runs\' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '-' + [guid]::NewGuid().ToString('N').Substring(0,8))
New-Item -ItemType Directory -Force -Path $evidence | Out-Null

function Invoke-YzbotRemote([string]$Command) {
    # -m preserves nested Python/shell quotes in Windows PowerShell 5.1 and 7.
    $commandFile = [IO.Path]::GetTempFileName()
    try {
        [IO.File]::WriteAllText($commandFile, $Command.Replace("`r`n", "`n") + "`n", [Text.UTF8Encoding]::new($false))
        & $plink @sshOptions -m $commandFile
        if ($LASTEXITCODE -ne 0) { throw "VM 命令失败，退出码 $LASTEXITCODE；请查看上方错误。" }
    } finally {
        [IO.File]::Delete($commandFile)
    }
}

Start-Transcript -Path (Join-Path $evidence 'console.log') | Out-Null
try {
    if (-not (Test-Path -LiteralPath $plink)) { throw '找不到 PuTTY plink。' }
    Invoke-YzbotRemote "python3 -c `"print('SSH_PYTHON_QUOTING=PASS')`""
    $envCommand = 'source /opt/ros/humble/setup.bash && source ~/dev_ws/install/setup.bash && '
    $helper = '~/dev_ws/src/yzbot/tools_demo/scripts/video_submit.py'
    Invoke-YzbotRemote "test -r $helper && test -x ~/dev_ws/TMSCQtest_x86_x64.bin"
    if ($CheckOnly) { Write-Host 'CHECK_ONLY=PASS（未启动、未出题、未提交）'; return }
    if ($GeneratorOnly) {
        Invoke-YzbotRemote ($envCommand + "python3 $helper --generate-only")
        return
    }
    if (-not $PrepareOnly -and [string]::IsNullOrWhiteSpace($MappingRules)) {
        $MappingRules = Read-Host '粘贴本轮映射规则（例如 x/y 的颜色、数量对应区域；正式录制使用本轮指定规则）'
    }
    if (-not $PrepareOnly -and [string]::IsNullOrWhiteSpace($MappingRules)) { throw '没有映射规则，未启动。' }
    if (-not (Test-Path -LiteralPath $studio)) { throw '找不到 CoStudio。' }
    Write-Host '即将重启 VM 场景。已有任务执行时不要运行本命令。' -ForegroundColor Yellow
    if (-not (Get-Process -Name coStudio -ErrorAction SilentlyContinue)) {
        Start-Process -FilePath $studio -WindowStyle Normal
    }
    Invoke-YzbotRemote ('YZBOT_GAZEBO_GUI_RENDER_RATE=15 COSTUDIO_HEADLESS=False COSTUDIO_DEBUG_GUI=True ' +
        'COSTUDIO_RVIZ=True COSTUDIO_DASHBOARD=True COSTUDIO_CAMERA_STREAMS=True COSTUDIO_SLAM_EVIDENCE=True ' +
        'bash ~/dev_ws/src/yzbot/tools_demo/scripts/start_manual_problem_mode.sh')
    Write-Host "CoStudio 连接：ws://${VmAddress}:21274；布局：YZBot-20260921。" -ForegroundColor Cyan
    Write-Host '请同时展示 Gazebo 与 CoStudio；RViz 用于展示导航/建图。不要关闭 Ubuntu 图形桌面会话。'
    if ($PrepareOnly) { Write-Host 'PREPARE_ONLY=PASS（未出题、未提交）'; return }
    if (-not $AutoStart) {
        Read-Host '开始录屏/裁判开始计时后按回车：现场随机出题，发布一次并调用真实 API，成功后自动搬运' | Out-Null
    }
    $rules64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($MappingRules))
    Invoke-YzbotRemote ($envCommand + "python3 -u $helper --mapping-b64 '$rules64' --watch")
    Write-Host "过程证据已保存。主机日志：$evidence；机器人比赛结果以本轮真实终态和总墙钟为准。" -ForegroundColor Green
} finally {
    Stop-Transcript | Out-Null
}
