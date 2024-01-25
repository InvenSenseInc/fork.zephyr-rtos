@echo off

if exist "C:\Program Files\7-Zip\7z.exe" (
  set seven_zip="C:\Program Files\7-Zip\7z.exe"
) else (
  set seven_zip="C:\Program Files (x86)\7-Zip\7z.exe"
)

set __xian_drivers_version_=3.0.0-dev2

:: copy ICM drivers from frgnb-share01\SCM\SwdPackages
xcopy /Y \\frgnb-share01\SCM\SwdPackages\invn\firmware\emd-mcu\icm42670s_examples\atmel.arm-gnu-toolchain-cm4-fpu\%__xian_drivers_version_%\invn.firmware.emd-mcu.icm42670s_examples-atmel.arm-gnu-toolchain-cm4-fpu-%__xian_drivers_version_%.tar.gz tmp\

if not exist tmp\invn.firmware.emd-mcu.icm42670s_examples-atmel.arm-gnu-toolchain-cm4-fpu-%__xian_drivers_version_%.tar.gz (
  echo "Copy ICM-42670P drivers package from SwdPackages failed"
  pause
) else (
  :: un-tar.gz
  %seven_zip% -y e tmp\invn.firmware.emd-mcu.icm42670s_examples-atmel.arm-gnu-toolchain-cm4-fpu-%__xian_drivers_version_%.tar.gz -otmp\
  %seven_zip% -y x tmp\invn.firmware.emd-mcu.icm42670s_examples-atmel.arm-gnu-toolchain-cm4-fpu-%__xian_drivers_version_%.tar -otmp\Rep
  
  :: copy icm42670S drivers
  xcopy /S /I /Y tmp\Rep\sources\drivers\icm42670s\imu .\imu
  xcopy /S /I /Y tmp\Rep\sources\Invn\InvError.h .\Invn\InvError.h

  rmdir /S /Q tmp
)

