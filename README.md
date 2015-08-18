http://blog.hig.no/gtl/2014/08/28/opencv-and-android-studio/

Create new project with no activity.

Adding OpenCV to your new project

Create a folder called “libraries” inside your Android Studio project, and copy there entire content of sdk/java out of your OpenCV Android folder.
Rename that folder to “opencv”, so that you will end up having your Android Studio project with subfolder “libraries/opencv”.
Now, inside this “opencv” folder, create a build.gradle file, with the following content:

////////////////////////////////////////////////////////////////////////

apply plugin: 'android-library'

buildscript {
  repositories {
    mavenCentral()
  }
  dependencies {
    classpath 'com.android.tools.build:gradle:0.12.2'
  }
}

android {
  compileSdkVersion 19
  buildToolsVersion "19.1.0"

  defaultConfig {
    minSdkVersion 8
    targetSdkVersion 19
    versionCode 2490
    versionName "2.4.9"
  }

  sourceSets {
    main {
      manifest.srcFile 'AndroidManifest.xml'
      java.srcDirs = ['src']
      resources.srcDirs = ['src']
      res.srcDirs = ['res']
      aidl.srcDirs = ['src']
    }
  }
}

////////////////////////////////////////////////////////////////////////

add to settings.gradle:

include ':libraries:opencv'


Open Android Studio
Do this in Android Studio: Tools/Android/Sync Project with Grade files
Go to File/Project Structure, inside Modules pick your ‘app’, then from the Tab pick: Dependencies, click + to add new dependency, pick Module Dependency, and add :library:opencv dependency to your project. Click OK.
Create a jniLibs folder in the /app/src/main/ location and copy the all the folder with *.so files (armeabi, armeabi-v7a, mips, x86) in the jniLibs from the OpenCV Android SDK/native/libs folder.
Make sure that you do have Android SDK 19 installed (as per above gradle files), or use a version that you have installed.
Try to sync Gradle again, after adding the dependency.  You may need to delete section “android” from your top-level build.gradle if the sync complains.
Build the project.

Go to OpenCV Android SDK, pick a sample project that you’d like to try out.
First, delete the ‘res’ folder inside your own project app/src/main, then place the res folder from the samples inside your app/src/main folder.
First, delete the ‘java’ folder from app/src/main, then copy the ‘src’ folder from the samples in there (note, the src has to be renamed to java).
If you building example with native C++ files, you need to have NDK installed. Download it from Google’s developers portal, and add this line to your local.properties in the top-level of your project, below the sdk.dir line:

ndk.dir=/path/to/your/android-ndk



////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

MY ADDITIONS:

add to gradle.properties:

ndkDir=/home/ilhan/android-ndk-r10d
android.useDeprecatedNdk=true

add jni folder into app/src/main
