# Implementation Plan - Change JDK from 26 to 21

The user wants to change the project's JDK configuration from version 26 to version 21. Based on the project structure, this involves updating the IntelliJ IDEA project settings and the Gradle build configuration.

## Proposed Changes

### [IntelliJ IDEA Configuration]

#### [MODIFY] [.idea/misc.xml](file:///C:/projects/Biobuzz-v1-16205/.idea/misc.xml)
- Change `project-jdk-name` from "26" to "21".

### [Gradle Build Configuration]

#### [MODIFY] [build.common.gradle](file:///C:/projects/Biobuzz-v1-16205/Quickstart-master/build.common.gradle)
- Update `compileOptions` to use `JavaVersion.VERSION_21` for `sourceCompatibility` and `targetCompatibility`.

#### [MODIFY] [FtcRobotController/build.gradle](file:///C:/projects/Biobuzz-v1-16205/Quickstart-master/FtcRobotController/build.gradle)
- Update `compileOptions` to use `JavaVersion.VERSION_21` for `sourceCompatibility` and `targetCompatibility`.

## Verification Plan

### Manual Verification
- Verify that the project syncs successfully in Android Studio.
- Check `File > Project Structure > Project` to ensure the SDK is set to 21.
- Check `File > Settings > Build, Execution, Deployment > Build Tools > Gradle` to ensure the Gradle JDK is set to 21.
