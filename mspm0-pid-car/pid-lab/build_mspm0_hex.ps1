param(
    [string]$OutputHex = (Join-Path $PSScriptRoot 'Debug\pid_lab_mspm0.hex'),
    [string[]]$Defines = @()
)

$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path $PSScriptRoot).Path
$Sdk = 'C:\ti\mspm0_sdk_2_10_00_04'
$SysConfig = 'C:\ti\sysconfig_1.26.2\sysconfig_cli.bat'
$Compiler = 'C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmclang.exe'
$HexTool = 'C:\ti\ti_cgt_arm_llvm_4.0.2.LTS\bin\tiarmhex.exe'
$Build = Join-Path $Root 'tmp\build_hex'
Set-Location $Root

foreach ($tool in @($SysConfig, $Compiler, $HexTool)) {
    if (-not (Test-Path -LiteralPath $tool)) {
        throw "Missing build tool: $tool"
    }
}

New-Item -ItemType Directory -Force -Path $Build | Out-Null
$outputParent = Split-Path -Parent $OutputHex
if (-not [System.IO.Path]::IsPathRooted($OutputHex)) {
    $OutputHex = Join-Path $Root $OutputHex
    $outputParent = Split-Path -Parent $OutputHex
}
New-Item -ItemType Directory -Force -Path $outputParent | Out-Null

Write-Host '[1/4] Generate SysConfig files'
& $SysConfig --product "$Sdk\.metadata\product.json" --script (Join-Path $Root 'pid_lab.syscfg') --output (Join-Path $Root 'tmp\syscfg')
if ($LASTEXITCODE -ne 0) { throw 'SysConfig generation failed' }

$includes = @(
    '-I.',
    '-I.\tmp\syscfg',
    '-I.\SENSOR',
    '-I.\MOTOR',
    '-I.\ENCODER',
    '-I.\SERIAL',
    '-I.\LAB',
    '-I.\MPU',
    "-I$Sdk\source\third_party\CMSIS\Core\Include",
    "-I$Sdk\source"
)
$flags = @(
    '@.\tmp\syscfg\device.opt',
    '-O0', '-gdwarf-3', '-mcpu=cortex-m0plus', '-march=thumbv6m',
    '-mfloat-abi=soft', '-mthumb', '-Wall', '-Wextra'
)
foreach ($define in $Defines) {
    $flags += "-D$define"
}
$sources = @(
    'main.c',
    'MOTOR\motor.c',
    'ENCODER\encoder.c',
    'SERIAL\serial.c',
    'SENSOR\sensor.c',
    'MPU\imu.c',
    'LAB\lab_ctrl.c',
    'tmp\syscfg\ti_msp_dl_config.c'
)

Write-Host '[2/4] Compile MSPM0 sources'
$objects = @()
foreach ($source in $sources) {
    $objectName = $source.Replace([char]92, '_').Replace('/', '_').Replace('.c', '.o')
    $object = Join-Path $Build $objectName
    & $Compiler @flags @includes '-c' (Join-Path $Root $source) '-o' $object
    if ($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
    $objects += $object
}

$startup = Join-Path $Sdk 'source\ti\devices\msp\m0p\startup_system_files\ticlang\startup_mspm0g350x_ticlang.c'
$startupObject = Join-Path $Build 'startup_mspm0g350x_ticlang.o'
& $Compiler @flags "-I$Sdk\source\third_party\CMSIS\Core\Include" "-I$Sdk\source" '-c' $startup '-o' $startupObject
if ($LASTEXITCODE -ne 0) { throw 'Startup compile failed' }
$objects += $startupObject

Write-Host '[3/4] Link firmware'
$outFile = Join-Path $Build 'pid_lab_mspm0.out'
$mapFile = Join-Path $Build 'pid_lab_mspm0.map'
$xmlFile = Join-Path $Build 'pid_lab_mspm0_linkInfo.xml'
& $Compiler '-mcpu=cortex-m0plus' '-march=thumbv6m' '-mfloat-abi=soft' '-mthumb' @objects `
    (Join-Path $Root 'tmp\syscfg\device_linker.cmd') `
    (Join-Path $Root 'tmp\syscfg\device.cmd.genlibs') `
    "-L$Sdk\source" '-LC:\ti\ti_cgt_arm_llvm_4.0.2.LTS\lib' '-llibc.a' '-Wl,-c' `
    "-Wl,-m,$mapFile" "-Wl,--xml_link_info=$xmlFile" '-o' $outFile
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }

Write-Host '[4/4] Generate Intel HEX'
& $HexTool '--intel' '--memwidth=8' '--romwidth=8' '--outfile' $OutputHex $outFile
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }

$hexInfo = Get-Item -LiteralPath $OutputHex
Write-Host "Done: $($hexInfo.FullName) ($($hexInfo.Length) bytes)"
