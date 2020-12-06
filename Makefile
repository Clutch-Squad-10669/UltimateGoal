.PHONY: all build install clean

all: build

build:
	./gradlew build -x lint

install: build TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
	${HOME}/Android/platform-tools/adb install TeamCode/build/outputs/apk/debug/TeamCode-debug.apk

installDriverStation:
	${HOME}/Android/platform-tools/adb install doc/apk/FtcDriverStation-release.apk

clean:
	./gradlew clean
