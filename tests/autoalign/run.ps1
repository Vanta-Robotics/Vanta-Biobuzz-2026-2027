$ErrorActionPreference = 'Stop'
$autoAlignRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$autoAlignBuild = Join-Path $autoAlignRoot 'build\auto-align-tests'
$autoAlignJavaBin = if ($env:JAVA_HOME) { Join-Path $env:JAVA_HOME 'bin' } else { 'C:\Program Files\Android\Android Studio\jbr\bin' }
$autoAlignJavac = Join-Path $autoAlignJavaBin 'javac.exe'
$autoAlignJava = Join-Path $autoAlignJavaBin 'java.exe'
if (-not (Test-Path -LiteralPath $autoAlignJavac)) { throw 'Set JAVA_HOME to a JDK installation before running the tests.' }
New-Item -ItemType Directory -Path $autoAlignBuild -Force | Out-Null
$autoAlignSources = Join-Path $autoAlignRoot 'TeamCode\src\main\java\org\firstinspires\ftc\teamcode\vision\autoalign'
& $autoAlignJavac -d $autoAlignBuild (Join-Path $autoAlignSources 'AlignGeometry.java') (Join-Path $autoAlignSources 'AutoAlignConfig.java') (Join-Path $autoAlignSources 'AutoAlignController.java') (Join-Path $PSScriptRoot 'AutoAlignControllerTest.java')
if ($LASTEXITCODE -ne 0) { throw 'Auto-align test compilation failed.' }
& $autoAlignJava -cp $autoAlignBuild AutoAlignControllerTest
if ($LASTEXITCODE -ne 0) { throw 'Auto-align behavior checks failed.' }
