.PHONY: all build install clean

all: build

build:
	./gradlew build -x lint

install: build TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
	${HOME}/Android/platform-tools/adb install TeamCode/build/outputs/apk/debug/TeamCode-debug.apk


clean:
	./gradlew clean
