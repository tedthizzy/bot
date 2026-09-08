// The phone host (ADR-0013): a second host against the same gates, after the
// Pi passes them on hardware.  A skeleton only; see README.md.  Nothing here has
// been built: the machine this was written on has no Android SDK.
pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "rover-android"

include(":link", ":brain", ":ui")
