@echo off

if exist "C:\Program Files\7-Zip\7z.exe" (
  set seven_zip="C:\Program Files\7-Zip\7z.exe"
) else (
  set seven_zip="C:\Program Files (x86)\7-Zip\7z.exe"
)

set __icp101xx_drivers_version_=1.2.2-test1

:: copy ICP drivers from frgnb-share01\SCM\SwdPackages
xcopy /Y \\frgnb-share01\SCM\SwdPackages\invn\firmware\emd-smartmotion\ICP101xx\%__icp101xx_drivers_version_%\eMD-SmartMotion_ICP101xx_%__icp101xx_drivers_version_%.zip tmp\

if not exist tmp\eMD-SmartMotion_ICP101xx_%__icp101xx_drivers_version_%.zip (
  echo "Copy ICP-101xx drivers package from SwdPackages failed"
  pause
) else (
  :: un-tar.gz
  %seven_zip% -y x tmp\eMD-SmartMotion_ICP101xx_%__icp101xx_drivers_version_%.zip -otmp\
  
  :: copy ICP101xx drivers
  rmdir /Q /S .\src\Devices\Drivers\ICP101xx
  mkdir .\src\Devices\Drivers\ICP101xx
  xcopy /S /I /Y tmp\EMD-Core\sources\Invn\Devices\Drivers\Icp101xx .\src\Devices\Drivers\ICP101xx

  rmdir /S /Q tmp
)

