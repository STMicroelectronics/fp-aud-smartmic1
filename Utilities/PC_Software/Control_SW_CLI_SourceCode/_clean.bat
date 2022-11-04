call mingw32-make clean
del /S *.o
del /Q debug\*.*
del /Q release\moc*.*
del /Q Makefile*
del /Q object_script.*
del /Q ui_*.h
rmdir /s /q doc